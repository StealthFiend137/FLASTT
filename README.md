# FLASTT
Feedback Loop for AS5600 and TT motors

## Goal

To turn a pile of inexpensive parts from Ali Express into a servo as a component for other projects.

## Challenges

We're dealing with low cost microprocessors, so there are bound to be some constraints, limitations and trade offs.

### Running multiple AS5600s

When testing more than one servo, we'll need to communicate with multiple AS5600s. The AS5600 i2c address is hard-coded to `0x36`, as such, they cannot exist on the same i2c bus. So what are our options if we want to use more than one?

**Options where no additional hardware in needed**

- The RP2040 has 2 i2c buses so each one of these can be used to to connect to a single AS5600, thus giving up to 2 sensors.
- On the RP2040, the pins that are used by the i2c buses are configurable. It _may_ be possible to connect a different AS5600 sensor to a pair of i2c pins on the Pico. This would allow us to connect up to 5 devices per bus, giving me 10 "channels". This would require consuming 2 pins per sensor on the Pico. For a full 5 sensors per bus, this would require 10 pins. For a full 10 sensors this would required 20 pins. _Number of pins needed = number of AS56000 sensors * 2._
- I've since tried the using multiple pins, and discovered that it is actually possible, and furthermore at the clock (SCL) and data (SDA) pins do not need to be adjacent. As such, all five data (SDA) lines from the AS5600 sensors can connect to a single (SDA) pin on the pico, with only the clock lines needing independent pins. This would consume a 6 pins for a fully populated channel, 12 if both were fully populated. _Number of pins needed = (number of channels in use) + (number of sensors)_ 
- Programmable IO (PIO) May be another option for additiona i2c channels, there are two PIO blocks each of which has 4 state machines. These state machines could be used to generate additional i2c busses. It may be possible to use to up 8 additional i2c busses meaning we can use to up 10 AS5600 sensors, but it's more likley that we would need to use two for each channel due to DMA chaninging.

**Options that require additional hardware**

- An i2c multiplexer such as the TCA9548A would allow 8 AS5600s to be connected to each multiplexer. Each TCA9548A is also addressable via 3 address pins, meaning that 8 multiplexers can be connected per channel, permitting an absolutely rediculous 64 AS5600s.
- Additional Rapsberry Pi PICOs could be used.
- Something clever with transistors directing the clock and/or data signal to the desired AS5600 sensor.

## Hardware List

Ommitted from the following lists is the physcical design of the servo, or rather the modifcations needed to the gearbox to turn it into a servo. That will be addressed later though.

### Active components

- Processing power provided by the Raspberry Pi Pico 2040.
- Positional sensing provied by the AS5600, in i2c mode.
- An H-Bridge controller.

### Passive components 

- Movement provided by the little yellow Single Shaft "Smart Car TT Motor" gearbox and Brushed DC motor, available from literally everywhere.

## Gearbox Modification

