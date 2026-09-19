#![no_std]
#![no_main]

use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_usb_host::{descriptor::ConfigurationDescriptorChain, BusRoute, BusState};
use esp_hal::{
	timer::timg::TimerGroup,
	usb::otg::{embassy_usb_host::Driver, Usb},
};
use log::info;

// 使用这个宏来 定义固件信息
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
	esp_println::println!("init the esp32 s3");

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
	let (mut bus_ctrl, bus) = embassy_usb_host::bus(Driver::new(usb), &BUS_STATE);

	info!("usb ready!");

	let speed = bus_ctrl.wait_for_connection().await;

	info!("usb devices connect!: {:?}", speed);

	// 存贮 usb config 介绍
	let mut config_buf = [0u8; 256];

	// 枚举 USB 设备
	let result = bus
		.enumerate(BusRoute::Direct(speed), &mut config_buf)
		.await;

	match result {
		Ok((enum_info, config_len)) => {
			info!(
				"device enumerated: VID={:04x}, PID={:04x}, addr={}, config_len={}",
				enum_info.device_desc.vendor_id,
				enum_info.device_desc.product_id,
				enum_info.device_address,
				config_len,
			);

			let config_bytes = &config_buf[..config_len];

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

		Err(e) => {
			info!("enumeration error: {:?}", e);
		}
	}
}
