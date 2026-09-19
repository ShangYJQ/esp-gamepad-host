//! XInput (Xbox 360 有线协议) host class driver。
//!
//! 组织方式参考 embassy-usb-host 的 `class/gip.rs`：
//! 常量 → 公开类型 → 描述符查找 → 纯解析函数 → 驱动。
//!
//! 目前适配：飞智 黑武士 4 Pro 的 XInput 模式 (`045e:028e`)。
//!
//! # 输入报告格式
//!
//! | 偏移   | 内容                                         |
//! |--------|----------------------------------------------|
//! | 0      | 消息类型 (0x00 = 输入)                       |
//! | 1      | 长度 (0x14 = 20)                             |
//! | 2      | 按键 0: 方向键上/下/左/右, Start, Back, LS, RS |
//! | 3      | 按键 1: LB, RB, Guide, -, A, B, X, Y          |
//! | 4      | 左扳机 (0–255)                               |
//! | 5      | 右扳机 (0–255)                               |
//! | 6–7    | 左摇杆 X (i16 LE)                            |
//! | 8–9    | 左摇杆 Y (i16 LE)                            |
//! | 10–11  | 右摇杆 X (i16 LE)                            |
//! | 12–13  | 右摇杆 Y (i16 LE)                            |
//! | 14–19  | 保留                                         |
//!
//! 飞智 黑武士 4 Pro 实际每包发 32 字节，第 14–31 字节是它自己的扩展数据，这里先不解析。

use core::fmt;
use core::marker::PhantomData;

use embassy_usb_driver::host::{pipe, PipeError, UsbHostAllocator, UsbPipe};
use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType};
use embassy_usb_host::descriptor::ConfigurationDescriptorChain;
use embassy_usb_host::handler::EnumerationInfo;
use log::debug;

// ── XInput USB interface identifiers ─────────────────────────────────────────

const XINPUT_IFACE_CLASS: u8 = 0xFF; // Vendor-specific
const XINPUT_IFACE_SUBCLASS: u8 = 0x5D;
const XINPUT_IFACE_PROTOCOL: u8 = 0x01;
const TRANSFER_TYPE_INTERRUPT: u8 = 0x03;

// ── XInput message types ─────────────────────────────────────────────────────

const XINPUT_MSG_INPUT: u8 = 0x00;
const XINPUT_INPUT_LEN: u8 = 0x14; // 20 字节

/// 一包的最大长度 (全速中断端点的上限)。
pub const XINPUT_MAX_PACKET: usize = 64;

// ── Public types ─────────────────────────────────────────────────────────────

/// 解析后的 XInput 输入报告。
///
/// 数值都是协议原始范围：扳机 0–255，摇杆 −32768–32767。
/// 字段名和 upstream GIP 的 `GamepadReport` 保持一致。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GamepadReport {
	/// 方向键上。
	pub dpad_up: bool,
	/// 方向键下。
	pub dpad_down: bool,
	/// 方向键左。
	pub dpad_left: bool,
	/// 方向键右。
	pub dpad_right: bool,
	/// A 键。
	pub a: bool,
	/// B 键。
	pub b: bool,
	/// X 键。
	pub x: bool,
	/// Y 键。
	pub y: bool,
	/// 左肩键 (LB)。
	pub left_bumper: bool,
	/// 右肩键 (RB)。
	pub right_bumper: bool,
	/// 左摇杆按下 (LS / L3)。
	pub left_stick_press: bool,
	/// 右摇杆按下 (RS / R3)。
	pub right_stick_press: bool,
	/// Start 键。
	pub start: bool,
	/// Back 键。
	pub back: bool,
	/// Guide 键 (中间的 Xbox 键)。
	pub guide: bool,
	/// 左扳机 (0–255)。
	pub left_trigger: u8,
	/// 右扳机 (0–255)。
	pub right_trigger: u8,
	/// 左摇杆 X (−32768–32767，正数 = 右)。
	pub left_stick_x: i16,
	/// 左摇杆 Y (−32768–32767，正数 = 上)。
	pub left_stick_y: i16,
	/// 右摇杆 X (−32768–32767，正数 = 右)。
	pub right_stick_x: i16,
	/// 右摇杆 Y (−32768–32767，正数 = 上)。
	pub right_stick_y: i16,
}

impl fmt::Display for GamepadReport {
	/// 例如：`buttons=[A LB] LT=0 RT=255 L=(0, 0) R=(12000, -9000)`
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		let buttons = [
			(self.a, "A"),
			(self.b, "B"),
			(self.x, "X"),
			(self.y, "Y"),
			(self.left_bumper, "LB"),
			(self.right_bumper, "RB"),
			(self.left_stick_press, "LS"),
			(self.right_stick_press, "RS"),
			(self.start, "Start"),
			(self.back, "Back"),
			(self.guide, "Guide"),
			(self.dpad_up, "Up"),
			(self.dpad_down, "Down"),
			(self.dpad_left, "Left"),
			(self.dpad_right, "Right"),
		];

		write!(f, "buttons=[")?;
		let mut first = true;
		for (pressed, name) in buttons {
			if pressed {
				if !first {
					write!(f, " ")?;
				}
				f.write_str(name)?;
				first = false;
			}
		}
		write!(
			f,
			"] LT={} RT={} L=({}, {}) R=({}, {})",
			self.left_trigger,
			self.right_trigger,
			self.left_stick_x,
			self.left_stick_y,
			self.right_stick_x,
			self.right_stick_y,
		)
	}
}

/// XInput host class driver 的错误。
#[derive(Debug)]
pub enum XInputError {
	/// USB 传输错误 (设备拔出时是 `PipeError::Disconnected`)。
	Transfer(PipeError),
	/// 配置描述符里没有 XInput 接口。
	NoInterface,
	/// 没有空闲的 USB pipe。
	NoPipe,
}

impl From<PipeError> for XInputError {
	fn from(e: PipeError) -> Self {
		Self::Transfer(e)
	}
}

impl fmt::Display for XInputError {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		match self {
			Self::Transfer(e) => write!(f, "Transfer error: {:?}", e),
			Self::NoInterface => write!(f, "No XInput interface found"),
			Self::NoPipe => write!(f, "No free pipe"),
		}
	}
}

impl core::error::Error for XInputError {}

// ── Descriptor discovery ─────────────────────────────────────────────────────

/// 在配置描述符里找到的 XInput 接口信息。
///
/// 目前只有中断 IN 端点；OUT 端点等做 LED / 震动时再加。
#[derive(Clone, Debug)]
pub struct XInputInterfaceInfo {
	/// 中断 IN 端点地址 (带方向位，例如 0x81)。
	pub interrupt_in_ep: u8,
	/// 中断 IN 端点的最大包长。
	pub interrupt_in_mps: u16,
	/// 中断 IN 端点的轮询间隔 (来自端点描述符)。
	pub interrupt_in_interval: u8,
}

/// 在配置描述符里找 XInput 接口：vendor class 0xFF、subclass 0x5D、protocol 0x01。
pub fn find_xinput(config_desc: &[u8]) -> Option<XInputInterfaceInfo> {
	let cfg = ConfigurationDescriptorChain::try_from_slice(config_desc).ok()?;

	for iface in cfg.iter_interface() {
		if iface.interface_class != XINPUT_IFACE_CLASS
			|| iface.interface_subclass != XINPUT_IFACE_SUBCLASS
			|| iface.interface_protocol != XINPUT_IFACE_PROTOCOL
		{
			continue;
		}

		for ep in iface.iter_endpoints() {
			if ep.transfer_type() == TRANSFER_TYPE_INTERRUPT && ep.is_in() {
				return Some(XInputInterfaceInfo {
					interrupt_in_ep: ep.endpoint_address,
					interrupt_in_mps: ep.max_packet_size,
					interrupt_in_interval: ep.interval,
				});
			}
		}
	}

	None
}

// ── Standard XInput parsing ──────────────────────────────────────────────────

/// 解析一包标准 XInput 输入报告 (`00 14 ...`)；不是输入报告返回 `None`。
///
/// 格式见模块文档。只读前 20 字节，后面的扩展数据忽略。
pub fn parse_standard_input(data: &[u8]) -> Option<GamepadReport> {
	if data.len() < XINPUT_INPUT_LEN as usize || data[0] != XINPUT_MSG_INPUT || data[1] != XINPUT_INPUT_LEN {
		return None;
	}

	Some(GamepadReport {
		dpad_up: data[2] & (1 << 0) != 0,
		dpad_down: data[2] & (1 << 1) != 0,
		dpad_left: data[2] & (1 << 2) != 0,
		dpad_right: data[2] & (1 << 3) != 0,
		start: data[2] & (1 << 4) != 0,
		back: data[2] & (1 << 5) != 0,
		left_stick_press: data[2] & (1 << 6) != 0,
		right_stick_press: data[2] & (1 << 7) != 0,

		left_bumper: data[3] & (1 << 0) != 0,
		right_bumper: data[3] & (1 << 1) != 0,
		guide: data[3] & (1 << 2) != 0,
		a: data[3] & (1 << 4) != 0,
		b: data[3] & (1 << 5) != 0,
		x: data[3] & (1 << 6) != 0,
		y: data[3] & (1 << 7) != 0,

		left_trigger: data[4],
		right_trigger: data[5],

		left_stick_x: i16::from_le_bytes([data[6], data[7]]),
		left_stick_y: i16::from_le_bytes([data[8], data[9]]),
		right_stick_x: i16::from_le_bytes([data[10], data[11]]),
		right_stick_y: i16::from_le_bytes([data[12], data[13]]),
	})
}

// ── XInputHost driver ────────────────────────────────────────────────────────

/// XInput host class driver。
///
/// # 用法
///
/// 1. USB 枚举完成后调用 [`XInputHost::try_register`]。
/// 2. 循环调用 [`XInputHost::poll`] 读输入。
pub struct XInputHost<'d, A: UsbHostAllocator<'d>> {
	in_ch: A::Pipe<pipe::Interrupt, pipe::In>,
	_phantom: PhantomData<&'d ()>,
}

impl<'d, A: UsbHostAllocator<'d>> XInputHost<'d, A> {
	/// 为刚枚举好的设备创建驱动：在配置描述符里找 XInput 接口，打开中断 IN 管道。
	///
	/// Xbox 360 有线协议没有 GIP 那样的握手，这里不需要发任何初始化包。
	///
	/// # Errors
	///
	/// - [`XInputError::NoInterface`]：描述符里没有 XInput 接口。
	/// - [`XInputError::NoPipe`]：没有空闲的 pipe。
	pub async fn try_register(
		alloc: &A,
		config_desc: &[u8],
		enum_info: &EnumerationInfo,
	) -> Result<Self, XInputError> {
		let info = find_xinput(config_desc).ok_or(XInputError::NoInterface)?;

		let in_ep_info = EndpointInfo {
			addr: EndpointAddress::from_parts((info.interrupt_in_ep & 0x0F) as usize, Direction::In),
			ep_type: EndpointType::Interrupt,
			max_packet_size: info.interrupt_in_mps,
			interval_ms: info.interrupt_in_interval,
		};

		let in_ch = alloc
			.alloc_pipe::<pipe::Interrupt, pipe::In>(enum_info.device_address, &in_ep_info, enum_info.split())
			.map_err(|_| XInputError::NoPipe)?;

		Ok(Self {
			in_ch,
			_phantom: PhantomData,
		})
	}

	/// 读下一包输入报告。
	///
	/// 不是输入报告的包 (比如刚连上时的 LED 状态包 `01 03 xx`) 在内部跳过，
	/// 直到读到一包有效的输入才返回。
	pub async fn poll(&mut self) -> Result<GamepadReport, XInputError> {
		let mut buf = [0u8; XINPUT_MAX_PACKET];
		loop {
			let n = self.in_ch.request_in(&mut buf).await?;
			if let Some(report) = parse_standard_input(&buf[..n]) {
				return Ok(report);
			}
			debug!("XInput: skip non-input packet ({}B): {:02x?}", n, &buf[..n]);
		}
	}
}
