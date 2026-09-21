//! 设备无关的手柄抽象：各家协议的 driver 解析完都归一到这里的类型。
//!
//! 具体协议在各自模块里：[`crate::xinput`]、[`crate::ds5`]。

use core::fmt;

use embassy_usb_driver::host::{PipeError, UsbHostAllocator};
use embassy_usb_host::class::hid::HidError;
use embassy_usb_host::handler::EnumerationInfo;

use crate::ds5::Ds5Host;
use crate::xinput::XInputHost;

/// 解析后的手柄输入报告。
///
/// 这是各家协议的「最大公约数」，数值沿用 XInput 的原始范围：扳机 0–255，
/// 摇杆 −32768–32767。各家 driver 负责把自己的原始单位换算过来，上层不用关心
/// 手柄是哪一家。字段名和 upstream GIP 的 `GamepadReport` 保持一致。
///
/// 某一家独有、这里放不下的数据 (比如 DS5 的触摸板和陀螺仪) 由各自的 driver
/// 单独提供，不往这里塞。
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
	/// A 键 (DS5: ✕)。
	pub a: bool,
	/// B 键 (DS5: ○)。
	pub b: bool,
	/// X 键 (DS5: □)。
	pub x: bool,
	/// Y 键 (DS5: △)。
	pub y: bool,
	/// 左肩键 (LB / L1)。
	pub left_bumper: bool,
	/// 右肩键 (RB / R1)。
	pub right_bumper: bool,
	/// 左摇杆按下 (LS / L3)。
	pub left_stick_press: bool,
	/// 右摇杆按下 (RS / R3)。
	pub right_stick_press: bool,
	/// Start 键 (DS5: Options)。
	pub start: bool,
	/// Back 键 (DS5: Create)。
	pub back: bool,
	/// Guide 键 (中间的 Xbox 键 / PS 键)。
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

// ── Errors ───────────────────────────────────────────────────────────────────

/// 手柄 driver 的错误，各家协议共用。
#[derive(Debug)]
pub enum GamepadError {
	/// USB 传输错误 (设备拔出时是 `PipeError::Disconnected`)。
	Transfer(PipeError),
	/// 这个设备不归这家 driver 管 (接口对不上、VID/PID 不匹配)，或者不支持这个操作
	/// (比如描述符里没有输出端点，就没法震动)。
	///
	/// 分发时靠它决定要不要往下试，所以和真正的错误分开。
	NotSupported,
	/// 没有空闲的 USB pipe。
	NoPipe,
}

impl From<PipeError> for GamepadError {
	fn from(e: PipeError) -> Self {
		Self::Transfer(e)
	}
}

impl From<HidError> for GamepadError {
	fn from(e: HidError) -> Self {
		match e {
			HidError::Transfer(e) => Self::Transfer(e),
			HidError::NoInterface => Self::NotSupported,
			HidError::NoPipe => Self::NoPipe,
		}
	}
}

impl fmt::Display for GamepadError {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		match self {
			Self::Transfer(e) => write!(f, "Transfer error: {:?}", e),
			Self::NotSupported => write!(f, "Unsupported device"),
			Self::NoPipe => write!(f, "No free pipe"),
		}
	}
}

impl core::error::Error for GamepadError {}

// ── Dispatch ─────────────────────────────────────────────────────────────────

/// 已识别的手柄，按协议分发。
///
/// 两家 driver 的 pipe 是不同的具体类型，`no_std` 下走 `dyn` 要 `async_fn_in_trait`
/// 加分配器，不划算，所以这里用 enum 分发。
pub enum Gamepad<'d, A: UsbHostAllocator<'d>> {
	/// XInput (Xbox 360 有线协议)。
	XInput(XInputHost<'d, A>),
	/// DualSense (PS5)。
	Ds5(Ds5Host<'d, A>),
}

impl<'d, A: UsbHostAllocator<'d>> Gamepad<'d, A> {
	/// 认一个刚枚举好的设备，认不出返回 [`GamepadError::NotSupported`]。
	///
	/// 先试 XInput：它的 vendor class 三元组 (`0xFF/0x5D/0x01`) 是唯一的，不会误判。
	/// DS5 用的是标准 HID class，只能靠 VID/PID 认，所以排在后面 —— 反过来的话，
	/// 将来接一个既报 HID 又报 XInput 的复合设备会走错分支。
	pub async fn try_register(
		alloc: &A,
		config_desc: &[u8],
		enum_info: &EnumerationInfo,
	) -> Result<Self, GamepadError> {
		match XInputHost::try_register(alloc, config_desc, enum_info).await {
			Ok(pad) => return Ok(Self::XInput(pad)),
			// 不是 XInput，继续往下试
			Err(GamepadError::NotSupported) => {}
			Err(e) => return Err(e),
		}

		match Ds5Host::try_register(alloc, config_desc, enum_info) {
			Ok(pad) => return Ok(Self::Ds5(pad)),
			Err(GamepadError::NotSupported) => {}
			Err(e) => return Err(e),
		}

		Err(GamepadError::NotSupported)
	}

	/// 这个手柄用的协议名，打日志用。
	pub fn protocol(&self) -> &'static str {
		match self {
			Self::XInput(_) => "XInput",
			Self::Ds5(_) => "DualSense",
		}
	}

	/// 读下一包输入报告。
	pub async fn poll(&mut self) -> Result<GamepadReport, GamepadError> {
		match self {
			Self::XInput(pad) => pad.poll().await,
			Self::Ds5(pad) => pad.poll().await,
		}
	}
}

/// 设备无关的输出控制。
impl<'d, A: UsbHostAllocator<'d>> Gamepad<'d, A> {
	// ── 通用控制 ─────────────────────────────────────────────────────────────
	//
	// 这里只放两家都有对应物的东西。DS5 独有的 (灯条 RGB、自适应扳机、麦克风灯)
	// 挂在 crate::ds5::Ds5Host 上，要用就 match 出 Gamepad::Ds5 分支。

	/// 设置双马达震动。
	///
	/// `strong` 是低频大马达，`weak` 是高频小马达，都是 0–255。
	///
	/// 手柄自己保持状态，**只在要改变时调用** —— 跟着每个输入包发会把总线流量翻倍，
	/// 而且两家的 OUT 端点 interval 都只有 4ms。
	///
	/// # Errors
	///
	/// [`GamepadError::NotSupported`]：这个设备没有可用的输出端点。
	pub async fn set_rumble(&mut self, strong: u8, weak: u8) -> Result<(), GamepadError> {
		match self {
			Self::XInput(pad) => pad.set_rumble(strong, weak).await,
			Self::Ds5(pad) => pad.set_rumble(strong, weak).await,
		}
	}

	/// 停掉所有震动。
	pub async fn stop_rumble(&mut self) -> Result<(), GamepadError> {
		self.set_rumble(0, 0).await
	}

	/// 玩家编号指示灯，0 = 全灭。
	///
	/// XInput 点亮环形灯的对应扇区 (支持 1–4)，DS5 点亮那排 5 颗灯里对应的组合
	/// (支持 1–5)。超出范围的编号一律全灭。
	///
	/// # Errors
	///
	/// [`GamepadError::NotSupported`]：这个设备没有可用的输出端点。
	pub async fn set_player_index(&mut self, index: u8) -> Result<(), GamepadError> {
		match self {
			Self::XInput(pad) => pad.set_player_index(index).await,
			Self::Ds5(pad) => pad.set_player_index(index).await,
		}
	}
}
