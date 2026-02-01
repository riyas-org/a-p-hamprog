The original hardware schematic from a-p programmer is kept here.
It work as LVP (low voltage Programmer).
You can also use a uno board.
Add boost converter and level shifter to mclr line

ARDUINO UNO / NANO                OPTOCOUPLER (PC817)
      +----------------+               +-------------------+
      |                |               |   (Inside Chip)   |
      |       **A3** |---[ 220R ]--->| 1 (Anode)  (Col) 4|<--- [ 13.5V + ]
      |        GND     |-------------->| 2 (Cath)   (Emi) 3|---+
      +----------------+               +-------------------+   |
                                                               |
           TARGET PIC (ICSP)                                   |
         +-------------------+             **10k Ohm** |
         | 1: CLK (PGC)      |<--- [ A0 ]  **Resistor** |
         | 2: DAT (PGD)      |<--- [ A1 ]      |               |
         | 3: VCC (+5V)      |                 |               |
         | 4: MCLR (Vpp)     |<----------------+---------------+
         | 5: GND            |<--- [ GND ]     |
         +-------------------+                 |
                                             (GND)

You basically need a circuit to switch 5v on arduino A3 to MCLR to 9.5v
Connect the 5volt from arduino via 220 ohm resistor to the led (ANODE) of optocoupler (PC817)
CATHODE of optocoupler to ground of arduino.
The emitter of optocoupler to the MCLR pin of PIC to program
The emitter also need a 10k resistor to ground 
Collector of optocupler to approx 9.5 volt (battery or boost converter)
An extra 0.5 volt (9.5 instead of 9v) is a dirty hack to overcome some peak loss from unwanted voltage divider
Am not an expert and this can be improved in several ways
I used a cheap mt3608 to the 5volt and carefully adjusted the potentiometer on it to get 9.5volt
For checking optocoupler, i temperorly connected 5volt to led side (anode) via 220 ohm resistor,
and checked voltage on emitter of transistor where 10k is connected to get slightly around 9.5volt(max) 
