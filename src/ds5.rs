//! DualSense (PS5 手柄) USB host class driver。
//!
//! 组织方式和 [`crate::xinput`] 一致：常量 → 公开类型 → 解析 → 驱动。
//!
//! DS5 插 USB 时是个标准 HID 设备 (interface class 0x03)，管道分配和控制请求
//! 直接复用 upstream 的 [`HidHost`]，这里只管「认设备 + 解析字节」。
//!
//! 和蓝牙不同，USB 下不需要任何握手：枚举完就一直收 report ID 0x01 的 64 字节包
//! (蓝牙要先读 feature report 0x05 才会从兼容模式切到全量的 0x31 模式)。
//!
//! # 输入报告格式 (report ID 0x01，USB 下共 64 字节)
//!
//! | 偏移   | 内容                                                |
//! |--------|-----------------------------------------------------|
//! | 0      | 报告 ID (0x01 = 输入)                               |
//! | 1–2    | 左摇杆 X / Y (u8，0x80 居中，Y 正 = 下)             |
//! | 3–4    | 右摇杆 X / Y                                        |
//! | 5–6    | L2 / R2 模拟量 (0–255)                              |
//! | 7      | 序号 (每包递增)                                     |
//! | 8      | 按键 0: 低 4 bit = 方向键 hat；bit4–7 = □ ✕ ○ △     |
//! | 9      | 按键 1: L1, R1, L2, R2, Create, Options, L3, R3     |
//! | 10     | 按键 2: bit0 = PS, bit1 = 触摸板按下, bit2 = 麦克风 |
//! | 11–15  | 厂商数据 / 保留                                     |
//! | 16–21  | 陀螺仪 X / Y / Z (i16 LE)                           |
//! | 22–27  | 加速度计 X / Y / Z (i16 LE)                         |
//! | 28–31  | 传感器时间戳 (u32 LE)                               |
//! | 33–40  | 触摸点 ×2，每点 4 字节                              |
//! | 41–52  | 保留                                                |
//! | 53     | 状态: 低 4 bit 电量档位，高 4 bit 充电状态          |
//! | 54–63  | 保留                                                |
//!
//! 偏移取自 Linux `hid-playstation.c` 的 `struct dualsense_input_report`，
//! **还没在实机上逐字节核对过**。

use core::fmt;

use embassy_usb_driver::host::UsbHostAllocator;
use embassy_usb_host::class::hid::HidHost;
use embassy_usb_host::handler::EnumerationInfo;
use log::debug;

use crate::gamepad::{GamepadError, GamepadReport};

// ── Device identifiers ───────────────────────────────────────────────────────

/// Sony Interactive Entertainment。
const SONY_VID: u16 = 0x054C;
/// DualSense。
const DUALSENSE_PID: u16 = 0x0CE6;
/// DualSense Edge。
const DUALSENSE_EDGE_PID: u16 = 0x0DF2;

// ── Input report ─────────────────────────────────────────────────────────────

/// USB 下唯一的输入报告 ID。
const DS5_INPUT_REPORT_ID: u8 = 0x01;
/// 解析基础手柄状态最少要读到的字节数 (到按键 2 为止)。
const DS5_BASE_LEN: usize = 11;
/// 解析扩展数据最少要读到的字节数 (到状态字节为止)。
const DS5_EXTRA_LEN: usize = 54;

/// 一包的最大长度 (USB 下固定 64 字节)。
pub const DS5_MAX_PACKET: usize = 64;

// 按键 0 (偏移 8)
const BTN0_HAT: u8 = 0x0F;
const BTN0_SQUARE: u8 = 1 << 4;
const BTN0_CROSS: u8 = 1 << 5;
const BTN0_CIRCLE: u8 = 1 << 6;
const BTN0_TRIANGLE: u8 = 1 << 7;

// 按键 1 (偏移 9)。bit2 / bit3 是 L2 / R2 的数字量，和偏移 5–6 的模拟量重复，不用。
const BTN1_L1: u8 = 1 << 0;
const BTN1_R1: u8 = 1 << 1;
const BTN1_CREATE: u8 = 1 << 4;
const BTN1_OPTIONS: u8 = 1 << 5;
const BTN1_L3: u8 = 1 << 6;
const BTN1_R3: u8 = 1 << 7;

// 按键 2 (偏移 10)
const BTN2_PS: u8 = 1 << 0;
const BTN2_TOUCHPAD: u8 = 1 << 1;
const BTN2_MIC_MUTE: u8 = 1 << 2;

// 触摸点的第一个字节
const TOUCH_INACTIVE: u8 = 1 << 7;
const TOUCH_ID: u8 = 0x7F;

// 状态字节 (偏移 53)
const STATUS_BATTERY: u8 = 0x0F;
const STATUS_CHARGE_SHIFT: u8 = 4;

// ── Public types ─────────────────────────────────────────────────────────────

/// 触摸板上的一个触点。
///
/// 触摸板分辨率 1920 × 1080，原点在左上角。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TouchPoint {
	/// 这个触点上是否有手指。
	pub active: bool,
	/// 触点编号，手指抬起再按下会变。
	pub id: u8,
	/// X 坐标 (0–1919)。
	pub x: u16,
	/// Y 坐标 (0–1079)。
	pub y: u16,
}

/// 充电状态 (状态字节的高 4 bit)。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ChargeState {
	/// 用电池供电。
	#[default]
	Discharging,
	/// 正在充电。
	Charging,
	/// 已充满。
	Full,
	/// 温度异常，暂停充电。
	TemperatureError,
	/// 充电故障。
	ChargingError,
	/// 没见过的取值。
	Unknown(u8),
}

/// DS5 特有、[`GamepadReport`] 里放不下的数据。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ds5Extra {
	/// 触摸板按下 (整块触摸板就是一个按键)。
	pub touchpad_press: bool,
	/// 麦克风静音键。
	pub mic_mute: bool,
	/// 触摸板上的两个触点。
	pub touch: [TouchPoint; 2],
	/// 陀螺仪 X / Y / Z (原始值，未做标定)。
	pub gyro: [i16; 3],
	/// 加速度计 X / Y / Z (原始值，未做标定)。
	pub accel: [i16; 3],
	/// 传感器时间戳 (设备自己的计数器)。
	pub sensor_timestamp: u32,
	/// 电量百分比 (0–100)。
	pub battery_percent: u8,
	/// 充电状态。
	pub charge_state: ChargeState,
}

impl fmt::Display for Ds5Extra {
	/// 例如：`touch=[#0(960,540)] gyro=(1, -2, 0) accel=(0, 8192, 40) bat=75% Discharging`
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		write!(f, "touch=[")?;
		let mut first = true;
		for p in &self.touch {
			if p.active {
				if !first {
					write!(f, " ")?;
				}
				write!(f, "#{}({},{})", p.id, p.x, p.y)?;
				first = false;
			}
		}
		write!(f, "]")?;

		if self.touchpad_press {
			write!(f, " Pad")?;
		}
		if self.mic_mute {
			write!(f, " Mic")?;
		}

		write!(
			f,
			" gyro=({}, {}, {}) accel=({}, {}, {}) bat={}% {:?}",
			self.gyro[0],
			self.gyro[1],
			self.gyro[2],
			self.accel[0],
			self.accel[1],
			self.accel[2],
			self.battery_percent,
			self.charge_state,
		)
	}
}

// ── Device matching ──────────────────────────────────────────────────────────

/// 这个设备是不是 DualSense。
///
/// DS5 用的是标准 HID class，光看接口描述符和别的 HID 手柄分不开，只能认 VID/PID。
pub fn is_dualsense(enum_info: &EnumerationInfo) -> bool {
	let desc = &enum_info.device_desc;
	desc.vendor_id == SONY_VID && matches!(desc.product_id, DUALSENSE_PID | DUALSENSE_EDGE_PID)
}

// ── Parsing ──────────────────────────────────────────────────────────────────

/// u8 摇杆轴 (0x80 居中) 换算成 [`GamepadReport`] 的 i16 范围。
///
/// DS5 原始只有 8 位精度，换算后最低位步进是 256，这是协议本身的量化损失。
fn stick_axis(v: u8) -> i16 {
	(v as i16 - 128) * 256
}

/// 同上，但方向取反：DS5 的 Y 轴向下为正，[`GamepadReport`] 向上为正。
fn stick_axis_inverted(v: u8) -> i16 {
	// 取反前先钳到 -32767：i16::MIN 取反会溢出，而这个项目 release 也开着 debug-assertions
	let v = stick_axis(v).max(-i16::MAX);
	-v
}

/// 方向键 hat 展开成四个方向，返回 `(上, 下, 左, 右)`。
///
/// 0 = 上，顺时针每 45° 加一，8 = 松开。
fn decode_hat(hat: u8) -> (bool, bool, bool, bool) {
	match hat {
		0 => (true, false, false, false),
		1 => (true, false, false, true),
		2 => (false, false, false, true),
		3 => (false, true, false, true),
		4 => (false, true, false, false),
		5 => (false, true, true, false),
		6 => (false, false, true, false),
		7 => (true, false, true, false),
		// 8 = 松开；其余取值不该出现
		_ => (false, false, false, false),
	}
}

/// 解析一个触摸点 (4 字节：contact, x_lo, x_hi|y_lo, y_hi)。
///
/// X / Y 各 12 bit：第三个字节低 4 bit 是 X 的高位，高 4 bit 是 Y 的低位。
fn parse_touch(p: &[u8]) -> TouchPoint {
	TouchPoint {
		active: p[0] & TOUCH_INACTIVE == 0,
		id: p[0] & TOUCH_ID,
		x: (((p[2] & 0x0F) as u16) << 8) | p[1] as u16,
		y: ((p[3] as u16) << 4) | (p[2] >> 4) as u16,
	}
}

/// 状态字节 → `(电量百分比, 充电状态)`。
///
/// 低 4 bit 是 0–10 的电量档位，换算方式和 Linux `hid-playstation` 一致。
fn decode_battery(status: u8) -> (u8, ChargeState) {
	let level = (status & STATUS_BATTERY) as u16;
	let percent = (level * 10 + 5).min(100) as u8;

	match status >> STATUS_CHARGE_SHIFT {
		0x0 => (percent, ChargeState::Discharging),
		0x1 => (percent, ChargeState::Charging),
		0x2 => (100, ChargeState::Full),
		0xA | 0xB => (0, ChargeState::TemperatureError),
		0xF => (0, ChargeState::ChargingError),
		other => (0, ChargeState::Unknown(other)),
	}
}

/// 解析一包 DS5 输入报告里的基础手柄状态；不是输入报告返回 `None`。
///
/// 格式见模块文档。触摸板、陀螺仪、电量这些在 [`parse_extra`] 里。
pub fn parse_input(data: &[u8]) -> Option<GamepadReport> {
	if data.len() < DS5_BASE_LEN || data[0] != DS5_INPUT_REPORT_ID {
		return None;
	}

	let (dpad_up, dpad_down, dpad_left, dpad_right) = decode_hat(data[8] & BTN0_HAT);

	Some(GamepadReport {
		dpad_up,
		dpad_down,
		dpad_left,
		dpad_right,

		a: data[8] & BTN0_CROSS != 0,
		b: data[8] & BTN0_CIRCLE != 0,
		x: data[8] & BTN0_SQUARE != 0,
		y: data[8] & BTN0_TRIANGLE != 0,

		left_bumper: data[9] & BTN1_L1 != 0,
		right_bumper: data[9] & BTN1_R1 != 0,
		left_stick_press: data[9] & BTN1_L3 != 0,
		right_stick_press: data[9] & BTN1_R3 != 0,
		start: data[9] & BTN1_OPTIONS != 0,
		back: data[9] & BTN1_CREATE != 0,

		guide: data[10] & BTN2_PS != 0,

		left_trigger: data[5],
		right_trigger: data[6],

		left_stick_x: stick_axis(data[1]),
		left_stick_y: stick_axis_inverted(data[2]),
		right_stick_x: stick_axis(data[3]),
		right_stick_y: stick_axis_inverted(data[4]),
	})
}

/// 解析 DS5 特有的那部分数据；包不够长或不是输入报告返回 `None`。
pub fn parse_extra(data: &[u8]) -> Option<Ds5Extra> {
	if data.len() < DS5_EXTRA_LEN || data[0] != DS5_INPUT_REPORT_ID {
		return None;
	}

	let (battery_percent, charge_state) = decode_battery(data[53]);

	Some(Ds5Extra {
		touchpad_press: data[10] & BTN2_TOUCHPAD != 0,
		mic_mute: data[10] & BTN2_MIC_MUTE != 0,
		touch: [parse_touch(&data[33..37]), parse_touch(&data[37..41])],
		gyro: [
			i16::from_le_bytes([data[16], data[17]]),
			i16::from_le_bytes([data[18], data[19]]),
			i16::from_le_bytes([data[20], data[21]]),
		],
		accel: [
			i16::from_le_bytes([data[22], data[23]]),
			i16::from_le_bytes([data[24], data[25]]),
			i16::from_le_bytes([data[26], data[27]]),
		],
		sensor_timestamp: u32::from_le_bytes([data[28], data[29], data[30], data[31]]),
		battery_percent,
		charge_state,
	})
}

// ── Ds5Host driver ───────────────────────────────────────────────────────────

/// DS5 host class driver。
///
/// # 用法
///
/// 1. USB 枚举完成后调用 [`Ds5Host::try_register`]。
/// 2. 循环调用 [`Ds5Host::poll`] 读输入；DS5 特有的数据在 [`Ds5Host::extra`]。
pub struct Ds5Host<'d, A: UsbHostAllocator<'d>> {
	hid: HidHost<'d, A>,
	extra: Ds5Extra,
}

impl<'d, A: UsbHostAllocator<'d>> Ds5Host<'d, A> {
	/// 为刚枚举好的设备创建驱动。
	///
	/// USB 下 DS5 不需要握手，把 HID 管道打开就能收包，所以这里不是 async。
	///
	/// # Errors
	///
	/// - [`GamepadError::NotSupported`]：VID/PID 不是 DualSense，或描述符里没有 HID 接口。
	/// - [`GamepadError::NoPipe`]：没有空闲的 pipe。
	pub fn try_register(alloc: &A, config_desc: &[u8], enum_info: &EnumerationInfo) -> Result<Self, GamepadError> {
		if !is_dualsense(enum_info) {
			return Err(GamepadError::NotSupported);
		}

		Ok(Self {
			hid: HidHost::new(alloc, config_desc, enum_info)?,
			extra: Ds5Extra::default(),
		})
	}

	/// 读下一包输入报告，顺带更新 [`Ds5Host::extra`]。
	///
	/// 不是输入报告的包在内部跳过，直到读到一包有效的输入才返回。
	pub async fn poll(&mut self) -> Result<GamepadReport, GamepadError> {
		let mut buf = [0u8; DS5_MAX_PACKET];
		loop {
			let n = self.hid.read(&mut buf).await?;
			if let Some(report) = parse_input(&buf[..n]) {
				// 短包 (理论上不会有) 只更新得了基础状态，扩展数据保持上一包
				if let Some(extra) = parse_extra(&buf[..n]) {
					self.extra = extra;
				}
				return Ok(report);
			}
			debug!("DS5: skip non-input packet ({}B): {:02x?}", n, &buf[..n]);
		}
	}

	/// 最近一包里的 DS5 特有数据 (触摸板、陀螺仪、电量……)。
	///
	/// 目前还没有消费者：[`GamepadReport`] 是各家协议的最大公约数，这些数据要等上层
	/// 真的用上 (触摸板当鼠标、电量显示之类) 再接。
	#[allow(dead_code)]
	pub fn extra(&self) -> &Ds5Extra {
		&self.extra
	}
}
