#![no_std]
#![no_main]

use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb_driver::host::{DeviceEvent, PipeError, UsbHostController};
use embassy_usb_driver::Speed;
use embassy_usb_host::{
	descriptor::ConfigurationDescriptorChain, BusController, BusHandle, BusRoute, BusState,
	EnumerationError,
};
use esp_hal::{
	timer::timg::TimerGroup,
	usb::otg::{embassy_usb_host::Driver, Usb},
};
use log::{info, warn};

// 使用这个宏来 定义固件信息
esp_bootloader_esp_idf::esp_app_desc!();

type HostCtrl = BusController<'static, Driver<'static>>;
type HostBus = BusHandle<'static, <Driver<'static> as UsbHostController<'static>>::Allocator>;

/// usb_event_task -> 主任务的 port 事件 (Connected / Disconnected)。
static PORT_EVENTS: Channel<CriticalSectionRawMutex, DeviceEvent, 8> = Channel::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
	info!("init the esp32 s3");

	// 初始化 log 环境
	esp_println::logger::init_logger_from_env();

	// 获得外围设备权限
	let peripherals = esp_hal::init(esp_hal::Config::default());

	// 获得 timg0 硬件时钟组
	let timg0 = TimerGroup::new(peripherals.TIMG0);
	esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

	let usb = Usb::new_fs(
		peripherals.USB_FS,
		// USB D+ → GPIO20
		peripherals.GPIO20,
		// USB D- → GPIO19
		peripherals.GPIO19,
	);

	// 创建状态
	static BUS_STATE: BusState = BusState::new();

	// 把 ESP32 USB 驱动接到 Embassy USB Host
	let (bus_ctrl, bus) = embassy_usb_host::bus(Driver::new(usb), &BUS_STATE);

	// 必须有一个任务一直 poll wait_for_device_event()
	// 只有它才会把 port 断开传给正在进行的 transfer
	spawner.spawn(usb_event_task(bus_ctrl).unwrap());

	info!("USB ready!");

	// 当前已枚举设备的地址
	// 设备拔出后要归还给 BusState
	let mut device_addr: Option<u8> = None;

	loop {
		// 阻塞直到等到到了 usb 连接
		match PORT_EVENTS.receive().await {
			DeviceEvent::Connected(speed) => {
				info!("usb devices connect!: {:?}", speed);
				if let Some(addr) = device_addr.take() {
					bus.free_address(addr);
				}
				device_addr = enumerate_device(&bus, speed).await;
			}
			DeviceEvent::Disconnected => {
				info!("usb device disconnected");
				//  释放这个 设备
				if let Some(addr) = device_addr.take() {
					bus.free_address(addr);
				}
			}
			other => info!("usb port event: {:?}", other),
		}
	}
}

/// 独占 BusController，一直 poll port 事件并转发给主任务。
#[embassy_executor::task]
async fn usb_event_task(mut bus_ctrl: HostCtrl) {
	loop {
		let ev = bus_ctrl.wait_for_device_event().await;
		// 用 try_send：队列满时不能停下来，否则就不再 poll 了
		if PORT_EVENTS.try_send(ev).is_err() {
			warn!("usb event queue full, dropped {:?}", ev);
		}
	}
}

/// 枚举刚连上的设备并打印描述符；成功时返回设备地址。
async fn enumerate_device(bus: &HostBus, speed: Speed) -> Option<u8> {
	// 存贮 usb config 介绍
	let mut config_buf = [0u8; 256];

	// 枚举 USB 设备
	match bus
		.enumerate(BusRoute::Direct(speed), &mut config_buf)
		.await
	{
		Ok((enum_info, config_len)) => {
			info!(
				"device enumerated: VID={:04x}, PID={:04x}, addr={}, config_len={}",
				enum_info.device_desc.vendor_id,
				enum_info.device_desc.product_id,
				enum_info.device_address,
				config_len,
			);
			print_config(&config_buf[..config_len]);
			Some(enum_info.device_address)
		}
		Err(EnumerationError::Transfer(PipeError::Disconnected)) => {
			// 设备在枚举途中断开 (飞智切换身份时就是这样)，等下一次 Connected 即可
			info!("device disconnected during enumeration");
			None
		}
		Err(e) => {
			warn!("enumeration error: {:?}", e);
			None
		}
	}
}

// 打印描述符
fn print_config(config_bytes: &[u8]) {
	let config = match ConfigurationDescriptorChain::try_from_slice(config_bytes) {
		Ok(config) => config,
		Err(e) => {
			info!("parse configuration failed: {:?}", e);
			return;
		}
	};

	info!(
		"configuration: interfaces={}, total_len={}, value={}",
		config.num_interfaces, config.total_len, config.configuration_value,
	);

	for interface in config.iter_interface() {
		info!(
			"interface: num={}, alt={}, class={:#04x}, subclass={:#04x}, protocol={:#04x}, endpoints={}",
			interface.interface_number,
			interface.alternate_setting,
			interface.interface_class,
			interface.interface_subclass,
			interface.interface_protocol,
			interface.num_endpoints,
		);

		for ep in interface.iter_endpoints() {
			info!(
				"  endpoint: addr={:#04x}, num={}, in={}, type={}, max_packet={}, interval={}",
				ep.endpoint_address,
				ep.ep_number(),
				ep.is_in(),
				ep.transfer_type(),
				ep.max_packet_size,
				ep.interval,
			);
		}
	}
}
