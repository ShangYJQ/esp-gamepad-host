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

use embassy_usb_driver::host::{pipe, UsbHostAllocator, UsbPipe};
use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType};
use embassy_usb_host::class::hid::HidHost;
use embassy_usb_host::descriptor::ConfigurationDescriptorChain;
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

// ── Output report ────────────────────────────────────────────────────────────

/// USB 下的输出报告 ID。
const DS5_OUTPUT_REPORT_ID: u8 = 0x02;
/// USB 下输出报告的总长 (含报告 ID)。
const DS5_OUTPUT_LEN: usize = 63;

// 输出报告里的偏移
const OUT_FLAG0: usize = 1;
const OUT_FLAG1: usize = 2;
const OUT_MOTOR_RIGHT: usize = 3;
const OUT_MOTOR_LEFT: usize = 4;
const OUT_MIC_LED: usize = 9;
const OUT_RIGHT_TRIGGER: usize = 11;
const OUT_LEFT_TRIGGER: usize = 22;
const OUT_FLAG2: usize = 39;
const OUT_LIGHTBAR_SETUP: usize = 42;
const OUT_LED_BRIGHTNESS: usize = 43;
const OUT_PLAYER_LEDS: usize = 44;
const OUT_LIGHTBAR_RGB: usize = 45;

// valid_flag0 (偏移 1)。没置使能位的字段会被整个忽略，而且不报错。
const FLAG0_COMPATIBLE_VIBRATION: u8 = 1 << 0;
const FLAG0_HAPTICS_SELECT: u8 = 1 << 1;
const FLAG0_RIGHT_TRIGGER: u8 = 1 << 2;
const FLAG0_LEFT_TRIGGER: u8 = 1 << 3;

// valid_flag1 (偏移 2)
const FLAG1_MIC_LED: u8 = 1 << 0;
const FLAG1_LIGHTBAR: u8 = 1 << 2;
const FLAG1_PLAYER_LEDS: u8 = 1 << 4;

// valid_flag2 (偏移 39)
const FLAG2_LIGHTBAR_SETUP: u8 = 1 << 1;
const FLAG2_COMPATIBLE_VIBRATION2: u8 = 1 << 2;

/// `lightbar_setup`：掐掉插上时的渐变动画。
const LIGHTBAR_SETUP_LIGHT_OUT: u8 = 1 << 1;

/// 一个扳机效果块的长度：1 字节模式 + 10 字节参数。
const TRIGGER_BLOCK_LEN: usize = 11;

// 扳机效果模式
const TRIGGER_MODE_OFF: u8 = 0x00;
const TRIGGER_MODE_RIGID: u8 = 0x01;
const TRIGGER_MODE_PULSE: u8 = 0x02;

/// 扳机行程被分成 10 个位置：0 = 松开，9 = 扣到底。
const TRIGGER_POS_MAX: u8 = 9;
/// 阻力力度的上限。
const TRIGGER_FORCE_MAX: u8 = 8;

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

/// 自适应扳机的阻力效果。
///
/// 扳机行程被分成 10 个位置 (0 = 松开，9 = 扣到底)，这些变体在描述阻力沿行程
/// 怎么分布。序列化成「1 字节模式 + 10 字节参数」的定长块，简单模式只用到头几个
/// 参数字节。
///
/// 超范围的数值在编码时会被钳到合法区间，不会 panic。
// Rigid / Pulse 目前没有构造点，同样是等消费者的 API
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TriggerEffect {
	/// 无阻力，扳机自由。
	#[default]
	Off,
	/// 从 `start` 位置开始一直有阻力，像拉硬弹簧。
	Rigid {
		/// 阻力起始位置 (0–9)。
		start: u8,
		/// 力度 (0–8)。
		force: u8,
	},
	/// 只在 `start..=end` 这段有阻力，冲过去就松掉，像枪扳机的击发感。
	Pulse {
		/// 起点 (0–8)。
		start: u8,
		/// 终点，必须在起点之后 (会被钳住)。
		end: u8,
		/// 力度 (0–8)。
		force: u8,
	},
}

impl TriggerEffect {
	/// 写进 11 字节的效果块 (1 字节模式 + 10 字节参数)，多余的参数字节清零。
	fn encode(self, dst: &mut [u8]) {
		let dst = &mut dst[..TRIGGER_BLOCK_LEN];
		dst.fill(0);
		match self {
			Self::Off => dst[0] = TRIGGER_MODE_OFF,
			Self::Rigid { start, force } => {
				dst[0] = TRIGGER_MODE_RIGID;
				dst[1] = start.min(TRIGGER_POS_MAX);
				dst[2] = force.min(TRIGGER_FORCE_MAX);
			}
			Self::Pulse { start, end, force } => {
				// 起点先留出一格，否则下面 clamp 的下界会超过上界
				let start = start.min(TRIGGER_POS_MAX - 1);
				dst[0] = TRIGGER_MODE_PULSE;
				dst[1] = start;
				dst[2] = end.clamp(start + 1, TRIGGER_POS_MAX);
				dst[3] = force.min(TRIGGER_FORCE_MAX);
			}
		}
	}
}

/// DS5 的完整输出状态。
///
/// 手柄自己保持状态，只在要改变时发，不需要周期性重发。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ds5Output {
	/// 低频大马达 (0–255)。
	pub rumble_strong: u8,
	/// 高频小马达 (0–255)。
	pub rumble_weak: u8,
	/// 灯条颜色 R / G / B。
	pub lightbar: [u8; 3],
	/// 那排 5 颗玩家灯的位图 (bit0–bit4)。
	pub player_leds: u8,
	/// 玩家灯亮度：0 = 最亮，数值越大越暗。
	pub led_brightness: u8,
	/// 麦克风键上的灯。
	pub mic_led: bool,
	/// 左扳机 (L2) 的阻力效果。
	pub left_trigger: TriggerEffect,
	/// 右扳机 (R2) 的阻力效果。
	pub right_trigger: TriggerEffect,
}

// ── Descriptor discovery ─────────────────────────────────────────────────────

/// HID 接口 class。
const USB_CLASS_HID: u8 = 0x03;
/// 中断传输类型。
const TRANSFER_TYPE_INTERRUPT: u8 = 0x03;

/// 在配置描述符里找 HID 接口的中断 OUT 端点，输出报告走它。
///
/// upstream 的 `find_hid` 只找 IN 端点 (它的实现里写死了 `ep.is_in()`)，`HidHost`
/// 里也没有 OUT 管道，所以这一段得自己走。
///
/// 和 `find_hid` 保持一致：只看第一个 HID 接口。
fn find_hid_out_endpoint(config_desc: &[u8]) -> Option<EndpointInfo> {
	let cfg = ConfigurationDescriptorChain::try_from_slice(config_desc).ok()?;
	let iface = cfg.iter_interface().find(|i| i.interface_class == USB_CLASS_HID)?;
	let ep = iface
		.iter_endpoints()
		.find(|ep| ep.transfer_type() == TRANSFER_TYPE_INTERRUPT && !ep.is_in())?;

	Some(EndpointInfo {
		addr: EndpointAddress::from_parts((ep.endpoint_address & 0x0F) as usize, Direction::Out),
		ep_type: EndpointType::Interrupt,
		max_packet_size: ep.max_packet_size,
		interval_ms: ep.interval,
	})
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

// ── Output encoding ──────────────────────────────────────────────────────────

/// 玩家编号 → 5 颗玩家灯的位图。
///
/// PS5 的习惯是居中亮：玩家 1 只亮中间那颗，玩家 2 亮两侧，依此类推。
/// 超出 1–5 的编号一律全灭。
fn player_led_bitmap(index: u8) -> u8 {
	match index {
		1 => 0b0_0100,
		2 => 0b0_1010,
		3 => 0b1_0101,
		4 => 0b1_1011,
		5 => 0b1_1111,
		_ => 0,
	}
}

/// 把输出状态拼成 63 字节的输出报告。
///
/// 每类字段都要连同对应的 valid flag 一起置位，否则手柄会静默忽略。这里一次把所有
/// 使能位都置上、发送完整状态 —— 手柄本来就保持状态，重复下发同样的值没有副作用。
fn build_output_report(out: &Ds5Output, buf: &mut [u8; DS5_OUTPUT_LEN]) {
	buf.fill(0);
	buf[0] = DS5_OUTPUT_REPORT_ID;

	// 振动：先把通路选到振动，再开经典振动模式。
	// 老固件认 valid_flag0 的 COMPATIBLE_VIBRATION，新固件认 valid_flag2 的 VIBRATION2，
	// 两个都置上兼容面最大 —— 不认的那一位会被忽略。
	buf[OUT_FLAG0] |= FLAG0_HAPTICS_SELECT | FLAG0_COMPATIBLE_VIBRATION;
	buf[OUT_FLAG2] |= FLAG2_COMPATIBLE_VIBRATION2;
	buf[OUT_MOTOR_LEFT] = out.rumble_strong;
	buf[OUT_MOTOR_RIGHT] = out.rumble_weak;

	// 灯条 / 玩家灯 / 麦克风灯
	buf[OUT_FLAG1] |= FLAG1_LIGHTBAR | FLAG1_PLAYER_LEDS | FLAG1_MIC_LED;
	buf[OUT_LIGHTBAR_RGB..OUT_LIGHTBAR_RGB + 3].copy_from_slice(&out.lightbar);
	buf[OUT_PLAYER_LEDS] = out.player_leds;
	buf[OUT_LED_BRIGHTNESS] = out.led_brightness;
	buf[OUT_MIC_LED] = u8::from(out.mic_led);

	// 自适应扳机
	buf[OUT_FLAG0] |= FLAG0_LEFT_TRIGGER | FLAG0_RIGHT_TRIGGER;
	out.right_trigger.encode(&mut buf[OUT_RIGHT_TRIGGER..]);
	out.left_trigger.encode(&mut buf[OUT_LEFT_TRIGGER..]);
}

// ── Ds5Host driver ───────────────────────────────────────────────────────────

/// DS5 host class driver。
///
/// # 用法
///
/// 1. USB 枚举完成后调用 [`Ds5Host::try_register`]。
/// 2. 循环调用 [`Ds5Host::poll`] 读输入；DS5 特有的数据在 [`Ds5Host::extra`]。
/// 3. 震动 / 灯条 / 扳机用各个 `set_*`，或者 [`Ds5Host::set_output`] 一次设完。
pub struct Ds5Host<'d, A: UsbHostAllocator<'d>> {
	hid: HidHost<'d, A>,
	/// 输出报告用；描述符里没有中断 OUT 端点时是 `None`。
	out_ch: Option<A::Pipe<pipe::Interrupt, pipe::Out>>,
	extra: Ds5Extra,
	output: Ds5Output,
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

		let hid = HidHost::new(alloc, config_desc, enum_info)?;

		// 没有 OUT 端点也能用，只是输出控制不可用
		let out_ch = match find_hid_out_endpoint(config_desc) {
			Some(out_ep_info) => Some(
				alloc
					.alloc_pipe::<pipe::Interrupt, pipe::Out>(
						enum_info.device_address,
						&out_ep_info,
						enum_info.split(),
					)
					.map_err(|_| GamepadError::NoPipe)?,
			),
			None => None,
		};

		Ok(Self {
			hid,
			out_ch,
			extra: Ds5Extra::default(),
			output: Ds5Output::default(),
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

/// DS5 的输出控制。
///
/// 目前还没有消费者：这些是给上层用的接口，等 `main.rs` 真的要震动 / 点灯时再接。
#[allow(dead_code)]
impl<'d, A: UsbHostAllocator<'d>> Ds5Host<'d, A> {
	// ── 通用控制 (由 crate::gamepad::Gamepad 转发) ────────────────────────────

	/// 设置双马达震动，`strong` 是低频大马达，`weak` 是高频小马达。
	pub async fn set_rumble(&mut self, strong: u8, weak: u8) -> Result<(), GamepadError> {
		let mut out = self.output;
		out.rumble_strong = strong;
		out.rumble_weak = weak;
		self.set_output(out).await
	}

	/// 玩家编号指示灯 (1–5，其余值全灭)。
	pub async fn set_player_index(&mut self, index: u8) -> Result<(), GamepadError> {
		let mut out = self.output;
		out.player_leds = player_led_bitmap(index);
		self.set_output(out).await
	}

	// ── DS5 独有 ─────────────────────────────────────────────────────────────

	/// 灯条颜色。
	///
	/// 手柄刚插上会播一段渐变动画，期间设的颜色会被盖掉；要立刻生效先调一次
	/// [`Ds5Host::stop_lightbar_animation`]。
	pub async fn set_lightbar(&mut self, r: u8, g: u8, b: u8) -> Result<(), GamepadError> {
		let mut out = self.output;
		out.lightbar = [r, g, b];
		self.set_output(out).await
	}

	/// 麦克风键上的灯。
	pub async fn set_mic_led(&mut self, on: bool) -> Result<(), GamepadError> {
		let mut out = self.output;
		out.mic_led = on;
		self.set_output(out).await
	}

	/// 两个扳机的阻力效果。
	///
	/// 这部分 Linux 内核完全没实现 (那 28 字节在内核里就叫 `reserved2`)，参数语义
	/// 是社区逆向的，手感对不对得上机实测。
	pub async fn set_trigger_effects(
		&mut self,
		left: TriggerEffect,
		right: TriggerEffect,
	) -> Result<(), GamepadError> {
		let mut out = self.output;
		out.left_trigger = left;
		out.right_trigger = right;
		self.set_output(out).await
	}

	/// 掐掉插上时的灯条渐变动画。
	///
	/// 单独发一包只带 `lightbar_setup` 的报告，不动当前输出状态。
	///
	/// **没在实机上验证过**：这一位的语义是社区逆向的，内核没碰。如果调完灯条直接
	/// 灭了而不是接管成功，那就是这一位理解反了，去掉这个调用即可。
	pub async fn stop_lightbar_animation(&mut self) -> Result<(), GamepadError> {
		let mut buf = [0u8; DS5_OUTPUT_LEN];
		buf[0] = DS5_OUTPUT_REPORT_ID;
		buf[OUT_FLAG2] = FLAG2_LIGHTBAR_SETUP;
		buf[OUT_LIGHTBAR_SETUP] = LIGHTBAR_SETUP_LIGHT_OUT;
		self.send(&buf).await
	}

	/// 一次性设置完整输出状态。
	///
	/// 分项的 setter 内部都走这里；要同时改好几项时直接调它，省掉多余的 USB 往返。
	pub async fn set_output(&mut self, out: Ds5Output) -> Result<(), GamepadError> {
		let mut buf = [0u8; DS5_OUTPUT_LEN];
		build_output_report(&out, &mut buf);
		self.send(&buf).await?;
		// 发成功了才记下来，失败时状态不至于和手柄对不上
		self.output = out;
		Ok(())
	}

	/// 当前的输出状态。
	pub fn output(&self) -> &Ds5Output {
		&self.output
	}

	/// 把输出报告发出去，走中断 OUT 端点。
	///
	/// 不能用控制端点的 `SET_REPORT`：实测 DS5 对它回 STALL。中断 OUT 也是 Linux
	/// `hid-playstation` 走的路径。
	///
	/// # Errors
	///
	/// [`GamepadError::NotSupported`]：描述符里没有中断 OUT 端点。
	async fn send(&mut self, buf: &[u8]) -> Result<(), GamepadError> {
		let out_ch = self.out_ch.as_mut().ok_or(GamepadError::NotSupported)?;
		out_ch.request_out(buf, true).await?;
		Ok(())
	}
}
