# ESP32-S3 xbox 手柄连接程序

#### 安装和编译

```bash
cargo install espup

espup install -t esp32s3

cargo +esp build --release

```

#### 烧录

```bash
brew install espflash
```

#### 板子信息

```bash
➜ espflash board-info
[2026-09-18T15:42:04Z INFO ] Serial port: '/dev/cu.wchusbserial1110'
[2026-09-18T15:42:04Z INFO ] Connecting...
[2026-09-18T15:42:05Z INFO ] Using flash stub
Chip type:         esp32s3 (revision v0.2)
Crystal frequency: 40 MHz
Flash size:        16MB
Features:          WiFi, BLE, Embedded Flash
MAC address:       b4:3a:45:ac:aa:20

Security Information:
=====================
Flags: 0x00000000 (0)
Key Purposes: [0, 0, 0, 0, 0, 0, 12]
Chip ID: 9
API Version: 0
Secure Boot: Disabled
Flash Encryption: Disabled
SPI Boot Crypt Count (SPI_BOOT_CRYPT_CNT): 0x0
```
