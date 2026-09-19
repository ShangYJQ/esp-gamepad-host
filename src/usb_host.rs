//! USB Host：USB 口本身的事情 —— 启动 bus、port 事件任务、枚举。
//!
//! 不涉及任何具体设备的协议：枚举完把 (枚举信息, 配置描述符) 交给上层的
//! class driver (比如 [`crate::xinput::XInputHost`])。

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb_driver::host::{DeviceEvent, PipeError, UsbHostController};
use embassy_usb_driver::Speed;
use embassy_usb_host::descriptor::ConfigurationDescriptorChain;
use embassy_usb_host::handler::EnumerationInfo;
use embassy_usb_host::{BusController, BusHandle, BusRoute, BusState, EnumerationError};
use esp_hal::usb::otg::{embassy_usb_host::Driver, Usb};
use log::{info, warn};

type HostCtrl = BusController<'static, Driver<'static>>;

/// 枚举设备、打开管道用的 bus 句柄。
pub type HostBus = BusHandle<'static, <Driver<'static> as UsbHostController<'static>>::Allocator>;

/// usb_event_task -> 上层的 port 事件 (Connected / Disconnected)。
static PORT_EVENTS: Channel<CriticalSectionRawMutex, DeviceEvent, 8> = Channel::new();

/// 启动 USB host：创建 bus，并启动一直 poll port 事件的任务。
pub fn start(spawner: Spawner, usb: Usb<'static>) -> HostBus {
	static BUS_STATE: BusState = BusState::new();

	// 把 ESP32 USB 驱动接到 Embassy USB Host
	let (bus_ctrl, bus) = embassy_usb_host::bus(Driver::new(usb), &BUS_STATE);

	// 必须有一个任务一直 poll wait_for_device_event()：
	// 只有它才会把 port 断开传给正在进行的 transfer
	spawner.spawn(usb_event_task(bus_ctrl).unwrap());

	bus
}

/// 等下一个 port 事件 (Connected / Disconnected)。
pub async fn next_event() -> DeviceEvent {
	PORT_EVENTS.receive().await
}

/// 独占 BusController，一直 poll port 事件并转发给上层。
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

/// 枚举刚连上的设备，配置描述符写进 `config_buf`。
///
/// 成功时返回枚举信息和描述符长度 (`&config_buf[..len]` 就是完整的配置描述符)。
pub async fn enumerate(bus: &HostBus, speed: Speed, config_buf: &mut [u8]) -> Option<(EnumerationInfo, usize)> {
	match bus.enumerate(BusRoute::Direct(speed), config_buf).await {
		Ok((enum_info, config_len)) => {
			info!(
				"device enumerated: VID={:04x}, PID={:04x}, addr={}, config_len={}",
				enum_info.device_desc.vendor_id,
				enum_info.device_desc.product_id,
				enum_info.device_address,
				config_len,
			);
			print_config(&config_buf[..config_len]);
			Some((enum_info, config_len))
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

/// 打印配置描述符里的接口和端点。
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
