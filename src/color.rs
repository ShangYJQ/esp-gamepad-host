//! 输入到颜色的映射：把摇杆的极坐标换算成 RGB。
//!
//! 摇杆是极坐标的 (方向 + 距离)，RGB 是三个独立通道，直接把轴接到通道上会变成
//! 三个互不相干的滑块，很难用。中间隔一层 HSV：色相本身就是个圆，和摇杆的旋转
//! 天然对应。
//!
//! 全整数实现。`no_std` 的 core 里没有 `atan2` 也没有 `sqrt`，用它们就得再拉一个
//! `libm` 依赖 —— 这里绕开了：角度用菱形角近似，开方用 `u32::isqrt`。

use crate::gamepad::GamepadReport;

/// 色相的取值范围：6 个扇区 × 256。
const HUE_MAX: u16 = 1536;

/// 摇杆方向 → 色相 (0..1536)。
///
/// 用「菱形角」近似代替 `atan2`：把角度量在 L1 单位圆 (菱形) 上而不是真正的圆上。
/// 和真实角度的偏差最大约 4.5°，对颜色看不出来。
///
/// 方向对应：右 = 红，上 = 黄绿，左 = 青，下 = 蓝紫。
fn stick_hue(x: i16, y: i16) -> u16 {
	let d = (x as i32).abs() + (y as i32).abs();
	if d == 0 {
		return 0;
	}
	let p = (y as i32) * (HUE_MAX as i32 / 4) / d;
	let a = if x < 0 {
		HUE_MAX as i32 / 2 - p
	} else if y < 0 {
		HUE_MAX as i32 + p
	} else {
		p
	};
	(a as u16) % HUE_MAX
}

/// 摇杆推出的距离 → 0..255。
fn stick_magnitude(x: i16, y: i16) -> u8 {
	let ax = (x as i32).unsigned_abs();
	let ay = (y as i32).unsigned_abs();
	// 必须在 u32 里算：两个 32768² 相加是 2147483648，比 i32::MAX 多一格，
	// 而这个项目 release 也开着 debug-assertions，溢出是真 panic 不是回绕
	let mag = (ax * ax + ay * ay).isqrt().min(i16::MAX as u32);
	(mag * 255 / i16::MAX as u32) as u8
}

/// 双极轴 (−32768..32767) → 0..255，居中约等于一半。
fn axis_to_level(v: i16) -> u8 {
	((v as i32 + 32768) * 255 / 65535) as u8
}

/// 整数 HSV → RGB。`h` 是 0..1536，`s` / `v` 是 0..255。
fn hsv_to_rgb(h: u16, s: u8, v: u8) -> (u8, u8, u8) {
	let (v, s) = (v as u32, s as u32);
	let sector = (h / 256) % 6;
	let f = (h % 256) as u32;

	let p = v * (255 - s) / 255;
	let q = v * (255 - s * f / 255) / 255;
	let t = v * (255 - s * (255 - f) / 255) / 255;

	let (r, g, b) = match sector {
		0 => (v, t, p),
		1 => (q, v, p),
		2 => (p, v, t),
		3 => (p, q, v),
		4 => (t, p, v),
		_ => (v, p, q),
	};
	(r as u8, g as u8, b as u8)
}

/// 摇杆 → 灯条颜色。
///
/// 左摇杆当色轮：方向选色相，推出距离定饱和度 (中心 = 白光，边缘 = 纯色)。
/// 右摇杆 Y 当亮度滑条：推到底 = 灭，居中 = 半亮，推到顶 = 最亮。
pub fn sticks_to_rgb(r: &GamepadReport) -> (u8, u8, u8) {
	let hue = stick_hue(r.left_stick_x, r.left_stick_y);
	let sat = stick_magnitude(r.left_stick_x, r.left_stick_y);
	let val = axis_to_level(r.right_stick_y);
	hsv_to_rgb(hue, sat, val)
}
