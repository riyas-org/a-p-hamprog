# PIC Programmer for 8-bit Devices (ATU-100 & HVP Enhanced)

This project is a specialized fork of the Arduino-based PIC16F1xxx programmer. It is modified specifically for amateur radio enthusiasts working on projects like the **ATU-100 antenna tuner**, which uses the **PIC16(L)F1938**.

It provides a low-cost, open-source solution for flashing PIC MCUs using a ubiquitous Arduino Uno, eliminating the need for a dedicated PICkit for one-off projects.


<p align="center">
  <a href="https://github.com/riyas-org/a-p-hamprog/releases/download/v1.2.1/pp3r_windows_binary.zip">
    <img src="https://img.shields.io/badge/DOWNLOAD-v1.2.1%20Release-blue?style=for-the-badge&logo=github&logoColor=white" alt="Download v1.1">
  </a>
</p>

---

## 🛠 Key Enhancements in this Fork
* **High Voltage Programming (HVP) Support**: Added a `-h` flag to trigger a 9.5V pulse on the MCLR pin via an optocoupler circuit. This is essential for chips where Low Voltage Programming (LVP) is disabled.
* **Firmware Dumping**: Includes a modification to read and dump existing firmware from the target PIC.
* **Radio Project Ready**: Specifically tested and optimized for the **PIC16(L)F1938** used in amateur radio kits.

---

## 🔌 Hardware Configuration

### 1. Standard Connections (LVP)
If your PIC has LVP enabled, connect your Arduino (Uno/Nano) as follows:

| Arduino Pin | AVR Pin | Target PIC Pin | Function |
| :--- | :--- | :--- | :--- |
| **GND** | GND | **GND** | Common Ground |
| **5V** | VDD | **VDD** | Optional Power |
| **A3** | PC3 | **MCLR** | Reset Line |
| **A1** | PC1 | **PGD** | Programming Data |
| **A0** | PC0 | **PGC** | Programming Clock |

### 2. High Voltage (HVP) Modification
To enable HVP support, integrate an optocoupler and a boost converter (e.g., MT3608) to the MCLR line:

1.  **Boost Converter**: Set output to **9.5V**.
2.  **Optocoupler**: Connect the Arduino **A3** (MCLR) output to the input side.
3.  **HVP Rail**: Connect the transistor side collector to the **9.5V** positive and the emitter to the target **MCLR** pin.
4.  **Pull-down**: Add a **10kΩ resistor** from the MCLR emitter to Ground to ensure the line pulls low when inactive.

---

## 💻 Software Usage

### Compilation
**Linux/macOS:**
`gcc -Wall pp3r.c -o pp3r`

**Windows (MinGW):**
`gcc -Wall pp3r.c -o pp3r`

### Programming Commands
To program a **PIC16F1938** (standard LVP):
`./pp3r -c /dev/ttyACM0 -t 16f1938 -s 2000 file.hex`

Reading/dumping hex On Windows 
`pp3r_x86_64.exe -r -s 2000 -c COM3 -v2 -t 16f1938`

Writing hex on Windows
`pp3r_x86_64.exe -s 2000 -c COM3 -v2 -t 16f1938 atu_100_fw_EXT_32_oled_lvp.hex`

To enable **High Voltage Mode** (triggers the A3 optocoupler):
`./pp3r -s 2000 -c /dev/ttyACM0 -h -t 16f1938 file.hex`

On windows, the order of parameter is a bit critical, e.s.p keep file name as last argument
But to read hex, keep -r dump_file_name.hex before -t 16f1938
`pp3r_x86_64.exe -r my_hex_dump.exe -s 2000 -c COM3 -v2 -t 16f1938`

### Key Options
* `-h`: Enable High Voltage Programming (HVP) pulse.
* `-c [PORT]`: Specify the serial port (e.g., COM30 or /dev/ttyACM0).
* `-t [DEVICE]`: Target PIC model.
* `-s [MS]`: Delay in milliseconds to allow the Arduino bootloader to timeout.

### Extra options
* `-e`: Skip eeprom write.
* `-k`: Only eeprom write, skips FLASH.
* `-n`: Skip FLASH verification.
  
### Graphical User Interface
A simple graphical user interface for windows can be found in the release pages, which does it easy to flas hex files to atu 100 and allowes eeprom editing.
Link: https://github.com/riyas-org/a-p-hamprog/releases/tag/v1.2.1

### Programming the arduino
Upload the firmware (under fw folder) to an arduino using regular methods (arduino software). Thr file/sketch is ppr.ino

---
![Test programmer breakoutboard](/hw/20260131_213336.jpg)




## ⚠️ Disclaimer
This is a community-shared modification provided for those who want to use it once in a while for kits like the ATU-100. It is provided "as-is" with no guarantees. Please verify your voltages before connecting to the target PIC to avoid damage.

---
*Original base project by jaromir-sukuba.*
