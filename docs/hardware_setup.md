# Hardware Setup Guide

## Cube Orange ↔ Raspberry Pi 4 UART
RPi 4          Cube Orange
GPIO 14 (TX) → TELEM2 RX (pin 3)
GPIO 15 (RX) ← TELEM2 TX (pin 2)
GND          → GND (pin 6)

## RPi4 UART configuration

1. Edit `/boot/firmware/config.txt`:
enable_uart=1
dtoverlay=disable-bt

2. Disable serial console:
```bash
sudo systemctl disable serial-getty@ttyS0.service
```

3. Set baud rate:
```bash
stty -F /dev/ttyAMA0 921600
```

## Camera mount

- PiCamera V2.1 mounted DOWNWARD-FACING
- Cable routed cleanly away from ESC/motor noise
- IMX219 sensor: 3280×2464 native, 1080p operational
