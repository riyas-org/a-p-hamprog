### ⚡ Safe HVP Wiring (Optocoupler Method)

Using an optocoupler ensures that the 9V used for HVP never reaches the Arduino pins, preventing accidental damage.

```text
       ARDUINO NANO/UNO                 OPTOCOUPLER (PC817)
      +----------------+               +-------------------+
      |                |               |   (Inside Chip)   |
      |   ISP_MCLR [C3]|---[ 220R ]--->| 1 (Anode)  (Col) 4|<---+-- [ 13.5V ]
      |            GND |-------------->| 2 (Cath)   (Emi) 3|----+
      +----------------+               +-------------------+    |
                                                                |
                                                                |
           TARGET PIC (ICSP)                                    |
         +-------------------+          10k Pull-Down           |
         | 1: CLK (PGC)      |<--- [From Arduino C0]            |
         | 2: DAT (PGD)      |<--- [From Arduino C1]            |
         | 3: VCC (+5V)      |                                  |
         | 4: MCLR (Vpp)     |<---------------------------------+
         | 5: GND            |                                  |
         +-------------------+                                  |
                   |                                            |
                   +------------------[ Resistor ]--------------+
                                         (10k)

```

### 📋 How it works:

1. **Arduino Side:** Pin **C3** drives the internal LED of the optocoupler through a **220Ω current-limiting resistor**.
2. **High Voltage Side:** The optocoupler acts as a switch. When the Arduino turns on the internal LED, the optocoupler connects the **13.5V Boost Converter** output to the PIC’s **MCLR** pin.
3. **Pull-Down Resistor:** A **10kΩ resistor** is connected between the PIC's MCLR pin and GND. This ensures that when the optocoupler is "OFF," the MCLR pin stays at 0V (Reset), and when it is "ON," it jumps to 8V (HVP Entry).

### 🚀 HVP Command

As the optocoupler logic effectively "inverts" or isolates the signal, use command below with -h
**[CODE]bash**
./pp3r -c /dev/ttyUSB0 -s 16f1938 -h -p atu_100_fw_EXT_32_oled_lvp.hex
**[CODE]**

### 4-Sentence Release/Description (as requested):

"This project is a hobbyist tool for flashing PIC16F1938 chips in ATU-100 tuners, based on the original `a-p-prog`. Factory-locked units often require an optocoupler-isolated 9.5V boost converter to bypass HVP-only programming"
