//! 板载 RGB 灯 (WS2812)。
//!
//! 别的模块用 [`set_level`] 发一个亮度 (0 = 灭，255 = 最亮)，
//! [`led_task`] 负责把它写到灯上。两边互不等待。

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::peripherals::RMT;
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal::Async;
use esp_hal_smartled::{buffer_size, color_order, RmtSmartLeds, WS2812_TIMING};
use log::warn;
use smart_leds::{gamma, SmartLedsWriteAsync, RGB8};

/// 板子上只有 1 颗灯；WS2812 的颜色顺序是 GRB。
type Led = RmtSmartLeds<'static, { buffer_size::<RGB8>(1) }, Async, RGB8, color_order::Grb>;

/// 最新的亮度。只保留最后一次的值，灯来不及更新时旧值会被直接覆盖。
static LEVEL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// 初始化 RMT 和灯，启动 [`led_task`]。
pub fn start(spawner: Spawner, rmt: RMT<'static>, pin: impl PeripheralOutput<'static>) {
	let freq = Rate::from_mhz(80);
	let rmt = Rmt::new(rmt, freq).unwrap().into_async();
	let led: Led = RmtSmartLeds::new(WS2812_TIMING, rmt.channel0, pin, freq).unwrap();

	spawner.spawn(led_task(led).unwrap());
}

/// 设置灯的亮度：0 = 灭，255 = 最亮。
pub fn set_level(level: u8) {
	LEVEL.signal(level);
}

/// 等新的亮度，把它写到灯上。
#[embassy_executor::task]
async fn led_task(mut led: Led) {
	loop {
		let level = LEVEL.wait().await;

		// 白光；gamma 校正让亮度变化在人眼看来更均匀
		let color = RGB8::new(level, level, level);
		if let Err(e) = led.write(gamma([color].into_iter())).await {
			warn!("led write failed: {:?}", e);
		}
	}
}
