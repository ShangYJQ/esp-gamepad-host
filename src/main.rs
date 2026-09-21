#![no_std]
#![no_main]

mod ds5;
mod gamepad;
#[cfg(feature = "rt-led")]
mod led;
mod usb_host;
mod xinput;

use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_usb_driver::host::{DeviceEvent, PipeError};
use embassy_usb_host::handler::EnumerationInfo;
use esp_hal::{timer::timg::TimerGroup, usb::otg::Usb};
use log::{info, warn};

use crate::gamepad::{Gamepad, GamepadError, GamepadReport};
use crate::usb_host::HostBus;

// 使用这个宏来 定义固件信息
esp_bootloader_esp_idf::esp_app_desc!();

/// 应用层的死区 (XInput.h 里的推荐值)。驱动给的是原始值，死区由应用决定。
const LEFT_STICK_DEADZONE: i16 = 7849;
const RIGHT_STICK_DEADZONE: i16 = 8689;
const TRIGGER_THRESHOLD: u8 = 30;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
	// 初始化 log 环境 (要放在第一条 info! 之前，否则打印不出来)
	esp_println::logger::init_logger_from_env();
	info!("init the esp32 s3");

	// 获得外围设备权限
	let peripherals = esp_hal::init(esp_hal::Config::default());

	// 获得 timg0 硬件时钟组
	let timg0 = TimerGroup::new(peripherals.TIMG0);
	esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

	// 板载 RGB 灯 (WS2812) 接在 GPIO48
	#[cfg(feature = "rt-led")]
	led::start(spawner, peripherals.RMT, peripherals.GPIO48);

	let usb = Usb::new_fs(
		peripherals.USB_FS,
		// USB D+ → GPIO20
		peripherals.GPIO20,
		// USB D- → GPIO19
		peripherals.GPIO19,
	);
	let bus = usb_host::start(spawner, usb);

	info!("USB ready!");

	// 当前已枚举设备的地址；设备拔出后要归还给 BusState
	let mut device_addr: Option<u8> = None;

	loop {
		match usb_host::next_event().await {
			DeviceEvent::Connected(speed) => {
				info!("usb device connected: {:?}", speed);
				if let Some(addr) = device_addr.take() {
					bus.free_address(addr);
				}

				// 配置描述符要一直留着，交给 class driver 找接口和端点
				// DS5 带内置音频，配置描述符里还有 UAC 接口，256 不够
				let mut config_buf = [0u8; 512];
				if let Some((enum_info, config_len)) = usb_host::enumerate(&bus, speed, &mut config_buf).await {
					device_addr = Some(enum_info.device_address);
					run_gamepad(&bus, &enum_info, &config_buf[..config_len]).await;
				}
			}
			DeviceEvent::Disconnected => {
				info!("usb device disconnected");
				if let Some(addr) = device_addr.take() {
					bus.free_address(addr);
				}
			}
			other => info!("usb port event: {:?}", other),
		}
	}
}

/// 如果设备是认得出的手柄 (XInput 或 DualSense)，就一直读输入，直到断开或出错。
async fn run_gamepad(bus: &HostBus, enum_info: &EnumerationInfo, config: &[u8]) {
	let mut pad = match Gamepad::try_register(bus, config, enum_info).await {
		Ok(pad) => pad,
		Err(GamepadError::NotSupported) => {
			info!("not a supported gamepad, ignored");
			return;
		}
		Err(e) => {
			warn!("gamepad register failed: {}", e);
			return;
		}
	};
	info!(
		"{} gamepad ready: VID={:04x} PID={:04x}",
		pad.protocol(),
		enum_info.device_desc.vendor_id,
		enum_info.device_desc.product_id,
	);

	let mut last = GamepadReport::default();
	loop {
		match pad.poll().await {
			Ok(report) => {
				// 只有状态变了才打印 (摇杆抖动已经被死区滤掉)
				let report = apply_deadzone(report);
				if report != last {
					info!("{}", report);
					// RT 控制板载灯：0 = 灭，255 = 最亮
					#[cfg(feature = "rt-led")]
					led::set_level(report.right_trigger);
					last = report;
				}
			}
			Err(GamepadError::Transfer(PipeError::Disconnected)) => {
				info!("gamepad disconnected");
				#[cfg(feature = "rt-led")]
				led::set_level(0);
				return;
			}
			Err(e) => {
				warn!("gamepad read error: {}, stop", e);
				#[cfg(feature = "rt-led")]
				led::set_level(0);
				return;
			}
		}
	}
}

/// 摇杆在死区内归零、扳机低于阈值归零，去掉静止时的抖动。
fn apply_deadzone(mut r: GamepadReport) -> GamepadReport {
	(r.left_stick_x, r.left_stick_y) = radial_deadzone(r.left_stick_x, r.left_stick_y, LEFT_STICK_DEADZONE);
	(r.right_stick_x, r.right_stick_y) = radial_deadzone(r.right_stick_x, r.right_stick_y, RIGHT_STICK_DEADZONE);
	if r.left_trigger < TRIGGER_THRESHOLD {
		r.left_trigger = 0;
	}
	if r.right_trigger < TRIGGER_THRESHOLD {
		r.right_trigger = 0;
	}
	r
}

/// 圆形死区：摇杆离中心的距离小于 `deadzone` 时两个轴都归零。
fn radial_deadzone(x: i16, y: i16, deadzone: i16) -> (i16, i16) {
	let (x2, y2, dz) = (x as i64, y as i64, deadzone as i64);
	if x2 * x2 + y2 * y2 < dz * dz {
		(0, 0)
	} else {
		(x, y)
	}
}
