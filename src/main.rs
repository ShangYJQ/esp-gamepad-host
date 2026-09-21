#![no_std]
#![no_main]

mod color;
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

/// 灯条颜色的量化步长。
///
/// 和 [`RUMBLE_STEP`] 同理：摇杆一动 report 每包都在变，不量化的话绕一圈色轮会发出
/// 上千个输出报告。
const LIGHTBAR_STEP: u8 = 16;

/// 震动幅度的量化步长。
///
/// 扳机是 0–255 的连续值，不量化的话从松到底推一次会发出上百个输出报告，和输入
/// 轮询抢同一条总线 (两家的 OUT 端点 interval 都只有 4ms)。
const RUMBLE_STEP: u8 = 16;

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

	// 连上就点亮玩家 1 指示灯，顺便探一下这个设备收不收输出报告
	let mut output_ok = true;
	if let Err(e) = pad.set_player_index(2).await {
		warn!("gamepad output unavailable: {}, rumble disabled", e);
		output_ok = false;
	}

	// DS5 独有：连上把灯条设成绿色。XInput 没有 RGB，所以这段只能在这个分支里
	if let Gamepad::Ds5(ds5) = &mut pad
		&& let Err(e) = ds5.set_lightbar(0, 255, 0).await
	{
		warn!("set lightbar failed: {}", e);
	}

	let mut last = GamepadReport::default();
	let mut last_rumble = 0u8;
	let mut players = 0u8;
	let mut last_rgb = [0u8; 3];
	loop {
		// 先把结果接出来：poll() 借用 pad，直接 match 的话臂里就没法再调 set_rumble
		let polled = pad.poll().await;
		match polled {
			Ok(report) => {
				// 只有状态变了才打印 (摇杆抖动已经被死区滤掉)
				let report = apply_deadzone(report);
				if report != last {
					info!("{}", report);
					// RT 控制板载灯：0 = 灭，255 = 最亮
					#[cfg(feature = "rt-led")]
					led::set_level(report.right_trigger);

					// LT 控制手柄震动幅度
					let level = rumble_level(report.left_trigger);
					if output_ok && level != last_rumble {
						if let Err(e) = pad.set_rumble(level, level).await {
							warn!("set rumble failed: {}, stop trying", e);
							output_ok = false;
						}
						last_rumble = level;
					}
					// 方向键上下调玩家灯档位。和 last 比一次，做成按一下走一格，
					// 否则按住不放时任何别的状态变化都会让它继续跳
					let step = if report.dpad_up && !last.dpad_up && players < 5 {
						1i8
					} else if report.dpad_down && !last.dpad_down && players > 0 {
						-1
					} else {
						0
					};
					if output_ok && step != 0 {
						players = players.saturating_add_signed(step);
						if let Err(e) = pad.set_player_index(players).await {
							warn!("set player index failed: {}, stop trying", e);
							output_ok = false;
						}
					}

					// 摇杆控制 DS5 灯条：左摇杆当色轮，右摇杆 Y 当亮度滑条
					let (red, green, blue) = color::sticks_to_rgb(&report);
					let rgb = [red / LIGHTBAR_STEP, green / LIGHTBAR_STEP, blue / LIGHTBAR_STEP];
					if output_ok && rgb != last_rgb {
						if let Gamepad::Ds5(ds5) = &mut pad
							&& let Err(e) = ds5.set_lightbar(red, green, blue).await
						{
							warn!("set lightbar failed: {}, stop trying", e);
							output_ok = false;
						}
						last_rgb = rgb;
					}

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
				// 读失败不代表写也失败，尽力停掉震动，别让手柄一直响下去
				let _ = pad.stop_rumble().await;
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

/// 扳机值 → 震动幅度，量化到 [`RUMBLE_STEP`] 一档。
fn rumble_level(trigger: u8) -> u8 {
	// 扣到底给满值，否则最高只到 240
	if trigger > u8::MAX - RUMBLE_STEP {
		u8::MAX
	} else {
		trigger / RUMBLE_STEP * RUMBLE_STEP
	}
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
