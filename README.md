# Stormbreaker
### ~~A ripoff of YveltalGriffin's Mjolnir~~ 
### A companion board for [Thundervolt](https://github.com/mackieks/thundervolt)

<img width="1918" height="921" alt="Stormbreaker PCB front" src="https://github.com/user-attachments/assets/423c1cf4-0417-421a-ae6f-dcc3a9eca25d" />

## What is this?
Stormbreaker is a multifunction circuit board intended to be used alongside [Thundervolt](https://github.com/mackieks/thundervolt) in Wii portables. It handles battery charging and monitoring, power switching, storage interfacing, and a bit more. See the full list of features here.

## Why did I design this?
Thundervolt, while incredible, does not provide any battery management capability, nor does it provide any way for a user to toggle power to the system outside of disconnecting power from Thundervolt itself. As such, following the completion of Thundervolt, its creator YveltalGriffin began work on a companion board, named "Mjolnir". Mjolnir would handle everything from battery management to storage to fan control and more, and would integrate seamlessly with Thundervolt. However, development on Mjolnir has been temporarily halted, which pushed me to make a companion board myself. Hence, Stormbreaker was born.

## Where did the name come from?
In Norse mythology, Thor is the God of Thunder, and is often seen alongside his hammer, Mjolnir. In the Marvel Cinematic Universe, Thor lost Mjolnir, and instead forged an axe called Stormbreaker. 

As such, it seemed fitting to name this board, born from the absense of Mjolnir, "Stormbreaker".
###### Disclaimer: this board is not capable of summoning the Bifrost. PRs are welcome.

## What can this board do?
Functionality is largely split between the 4 ICs.
#### U1 - BQ25895
- Battery charging
  - Fast charging capability
  - Play-and-charge capability
- Battery voltage monitoring
  - Used to determine approximate remaining battery capacity (i.e. "percent")
- Shipping mode
#### U2 - GL835
- microSD to USB 2.0 adapter
- Supports hot-swaping microSD cards
#### U3 - ATtiny1616
- Initialises ICs
- Monitors system
  - Shuts off console in event of over-temp
  - Engages shipping mode when requested
  - Responds to errors from U1
- Powers on/off console after button press
  - Has safe shutdown capability
- Drives Neopixel LED (WS2812 or compatible)
- Controls fan speed using PWM
- Communicates with Wii over I2C
  - RVLoader fork allows various adjustments, including charge currents and charge voltage
#### U4 - HUSB238A
- PD sink controller
- Uses USB-C PD to request higher voltages (up to 12V) to allow for faster charging with lower current draw
