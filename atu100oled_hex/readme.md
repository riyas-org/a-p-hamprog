# ⚡ pp3r: PIC Programmer for ATU-100

A "quick-n-dirty" weekend hobby project specifically designed for programming PIC microcontrollers in the **ATU-100 Antenna Tuner**.

**Credits:** This project is based on the original [a-p-prog by Jaromir Sukuba](https://github.com/jaromir-sukuba/a-p-prog).

## ⚠️ Important Disclaimer

* **NO GUARANTEE:** This is experimental software. Use at your own risk.
* **HEX DUMP WARNING:** Dumping/Reading hex files may have issues with **Config Bytes**. These often require manual correction in the HEX file after a dump.
* **VARIANT SUPPORT:** It may not handle all PIC variants correctly. This was built for the PIC16F1938.

## 🛠 Hardware & HVP vs LVP (Important!)

Many ATU-100 units from eBay or AliExpress arrive with:

1. **Code Protection:** Turned ON.
2. **HVP (High Voltage Programming):** Enabled/Mandatory.

### The Catch-22:

To program these factory-locked units for the first time, you **must use HVP mode** (requires a 9V boost converter on the MCLR pin). If you attempt to use HVP mode without a boost converter, or vice versa, the software will return **chip detection errors**.

**Good News:** The firmware provided in this repo has **LVP (Low Voltage Programming) enabled**. Once you successfully flash the chip once using HVP, all subsequent updates can be done in LVP mode (5V only) without the boost converter.

---

## 🚀 Useful Commands

### 1. Identify the Chip

Check if the programmer can see your PIC:

```bash
./pp3r -c /dev/ttyUSB0 -s 2000 -t  16f1938 -pn -v2

```

### 2. High Voltage Programming (HVP)

*Use this for factory-locked eBay/AliExpress units. Requires 9V boost circuit.*

```bash
./pp3r -c /dev/ttyUSB0 -s 2000 -t 16f1938 -h atu_100_fw_EXT_32_oled_lvp.hex -v2

```

* `-h`: Enables High Voltage mode.

### 3. Low Voltage Programming (LVP)

*Use this once you have flashed the chip with this project's firmware.*

```bash
./pp3r -c /dev/ttyUSB0 -s 2000 -t  16f1938  atu_100_fw_EXT_32_oled_lvp.hex -v2

```

### 4. Dumping/Reading a Chip

To attempt to backup existing firmware (remember to check config bytes manually!):

```bash
./pp3r -c /dev/ttyUSB0 -s 2000 -t  16f1938 -r dump.hex
```

### Windows Users

A bit glitchy with parameter ordering. Keep last parameter as ` -t 16f1938` if no hex to flash is supplied.
For reading hex, keep it as the first param `-r hex_dump.hex` 
Keep  `[hex dump file name][delay][com port][extras -nphke][chip name][file to flash]`


---

## 🔧 Installation

1. **Firmware:** Upload `ppr.ino` to an Arduino Nano or Uno using the Arduino IDE.
2. **Client:** Compile the C client on your PC:
```bash
gcc -Wall pp3r.c -o pp3r

```


3. **Connect:** Wire your Arduino to the PIC ICSP header (MCLR, DAT, CLK, GND, VCC).

## 🤝 Contributing

Feel free to modify, optimize, or fix timing issues. Pull requests are welcome! If you find a way to make the Config Byte dumping 100% reliable, please share the fix!
