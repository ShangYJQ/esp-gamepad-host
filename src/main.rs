#![no_std]
#![no_main]

use embassy_time::Timer;
use embassy_usb_host::handler::EnumerationInfo;
use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb_host::{
	descriptor::ConfigurationDescriptorChain, BusController, BusHandle, BusRoute, BusState,
	EnumerationError,
};
use esp_hal::{
	gpio::LpPin,
	timer::timg::TimerGroup,
	usb::otg::{embassy_usb_host::Driver, Usb},
};

use embassy_usb_driver::{
	host::{pipe, DeviceEvent, PipeError, UsbHostAllocator, UsbHostController, UsbPipe},
	Direction, EndpointAddress, EndpointInfo, EndpointType, Speed,
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
				// device_addr = enumerate_device(&bus, speed).await;

				if let Some(enum_info) = enumerate_device(&bus, speed).await {
					device_addr = Some(enum_info.device_address);

					if enum_info.device_desc.vendor_id == 0x045e
						&& enum_info.device_desc.product_id == 0x028e
					{
						info!("------- 识别到 飞智 黑武士 4pro 手柄 --------");
						read_xinput(&bus, &enum_info).await;
					}
				}
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
async fn enumerate_device(bus: &HostBus, speed: Speed) -> Option<EnumerationInfo> {
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
			Some(enum_info)
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

async fn read_xinput(bus: &HostBus, enum_info: &EnumerationInfo) {
	info!("进入 xinput 模式");

	// 这是刚才描述符里看到的 0x81：
	// endpoint 1、IN、Interrupt、64 bytes、1ms
	let ep = EndpointInfo {
		addr: EndpointAddress::from_parts(1, Direction::In),
		ep_type: EndpointType::Interrupt,
		max_packet_size: 64,
		interval_ms: 1,
	};

	// 为这个 endpoint 创建一条通信 pipe
	let mut pipe = match bus.alloc_pipe::<pipe::Interrupt, pipe::In>(
		enum_info.device_address,
		&ep,
		enum_info.split(),
	) {
		Ok(pipe) => pipe,
		Err(e) => {
			warn!("创建 XInput pipe 失败: {:?}", e);
			return;
		}
	};

	let mut buf = [0u8; 64];

	// 上一包的数据和长度，用来判断这一包有没有变化
	let mut last = [0u8; 64];
	let mut last_len = 0;

	loop {
		match pipe.request_in(&mut buf).await {
			Ok(n) => {
				// 直接读取手柄的 xpnut 包
				let data = &buf[..n];

				// 和上一包一样就跳过，只有数据变了才解析、打印
				if data == &last[..last_len] {
					continue;
				}
				last[..n].copy_from_slice(data);
				last_len = n;

				parse_xinput(data);
			}
			Err(PipeError::Disconnected) => {
				info!("手柄连接已经断开");
				return;
			}
			Err(e) => {
				info!("读取失败 {:?}", e);
			}
		}

		// Timer::after_millis(10).await;
	}
}

fn parse_xinput(data: &[u8]) {
	if data.len() < 14 {
		return;
	}

	if data[0] != 0x00 {
		info!("unknown xinput packet: {:02x?}", data);
		return;
	}

	let dpad_up = data[2] & 0x01 != 0;
	let dpad_down = data[2] & 0x02 != 0;
	let dpad_left = data[2] & 0x04 != 0;
	let dpad_right = data[2] & 0x08 != 0;

	let start = data[2] & 0x10 != 0;
	let back = data[2] & 0x20 != 0;
	let l3 = data[2] & 0x40 != 0;
	let r3 = data[2] & 0x80 != 0;

	let lb = data[3] & 0x01 != 0;
	let rb = data[3] & 0x02 != 0;
	let guide = data[3] & 0x04 != 0;

	let a = data[3] & 0x10 != 0;
	let b = data[3] & 0x20 != 0;
	let x = data[3] & 0x40 != 0;
	let y = data[3] & 0x80 != 0;

	let lt = data[4];
	let rt = data[5];

	let lx = i16::from_le_bytes([data[6], data[7]]);
	let ly = i16::from_le_bytes([data[8], data[9]]);
	let rx = i16::from_le_bytes([data[10], data[11]]);
	let ry = i16::from_le_bytes([data[12], data[13]]);

	info!(
		"A={} B={} X={} Y={} LB={} RB={} Guide={} \
		 Start={} Back={} L3={} R3={} \
		 DPad=[U:{} D:{} L:{} R:{}] \
		 LT={} RT={} LX={} LY={} RX={} RY={}",
		a,
		b,
		x,
		y,
		lb,
		rb,
		guide,
		start,
		back,
		l3,
		r3,
		dpad_up,
		dpad_down,
		dpad_left,
		dpad_right,
		lt,
		rt,
		lx,
		ly,
		rx,
		ry,
	);
}
