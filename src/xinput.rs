//! XInput (Xbox 360 有线协议) host class driver。
//!
//! 组织方式参考 embassy-usb-host 的 `class/gip.rs`：
//! 常量 → 公开类型 → 描述符查找 → 纯解析函数 → 驱动。
//!
//! 解析结果归一到 [`crate::gamepad::GamepadReport`]。
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

use core::marker::PhantomData;

use embassy_usb_driver::host::{pipe, UsbHostAllocator, UsbPipe};
use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType};
use embassy_usb_host::descriptor::ConfigurationDescriptorChain;
use embassy_usb_host::handler::EnumerationInfo;
use log::debug;

use crate::gamepad::{GamepadError, GamepadReport};

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
	/// - [`GamepadError::NotSupported`]：描述符里没有 XInput 接口。
	/// - [`GamepadError::NoPipe`]：没有空闲的 pipe。
	pub async fn try_register(
		alloc: &A,
		config_desc: &[u8],
		enum_info: &EnumerationInfo,
	) -> Result<Self, GamepadError> {
		let info = find_xinput(config_desc).ok_or(GamepadError::NotSupported)?;

		let in_ep_info = EndpointInfo {
			addr: EndpointAddress::from_parts((info.interrupt_in_ep & 0x0F) as usize, Direction::In),
			ep_type: EndpointType::Interrupt,
			max_packet_size: info.interrupt_in_mps,
			interval_ms: info.interrupt_in_interval,
		};

		let in_ch = alloc
			.alloc_pipe::<pipe::Interrupt, pipe::In>(enum_info.device_address, &in_ep_info, enum_info.split())
			.map_err(|_| GamepadError::NoPipe)?;

		Ok(Self {
			in_ch,
			_phantom: PhantomData,
		})
	}

	/// 读下一包输入报告。
	///
	/// 不是输入报告的包 (比如刚连上时的 LED 状态包 `01 03 xx`) 在内部跳过，
	/// 直到读到一包有效的输入才返回。
	pub async fn poll(&mut self) -> Result<GamepadReport, GamepadError> {
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
