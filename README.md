# DDS-915-DIY
Digital desoldering station (ZD-915 based)
- Using Arduino NANO board

> This scheme has been used since 2018, recently the OLED display was replaced, and some adjustments were made in the firmware to improve the temperature setpoint adjustment increment

### References

Watch these videos to understand how it works:

- [ZD-915 Desoldering Station review](https://www.youtube.com/watch?v=_Ar05rKqoEI)

- [ZD-915 Desoldering gun upgrade - overvoltage fix](https://www.youtube.com/watch?v=MPcmVaqe08Y)

Additional Information: [ZD-915 digital desoldering station](https://eleshop.eu/desolderingstation-zd-915.html)

### Software

- The Arduino sketch takes up almost all of the ATmega328's capacity

- PID type temperature control is used

- The biggest consumption of resources is the OLED display (0.96 inches) on the I2C bus

- The [temperature conversion](http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/thermocouple/type-e-calibration-table) of the thermocouple sensor is accurate, but not faithful. The gun has common wires and it would be necessary to use [cold junction sensors](http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/thermocouple/cold-junction-compensation), but it is suitable for non-commercial use

- In order to have some protection, the firmware may report two errors
- - Due to high temperature, or due to high heater current
- - - If the gun sensor informs a high temperature value, the power supply must turn off, and the pump is also activated in case the source cannot be turned off
- - - If the gun heater has a high current, the power supply is also turned off
- - - - So the ATX power supply turns out to be a good option to be adapted

### Hardware

- Schematic

![img](https://raw.githubusercontent.com/rtek1000/DDS-915-DIY/main/Hardware/Doc/DDS-915-DIY_Kicad.png)

- The power supply was adapted from an ATX computer power supply
- - The gun uses 24V, stronger diodes are needed to get -12V high power
- - - The +12V with the -12V form the 24V for the gun
- - - It is also necessary to adapt the filters
- - - Practically all ATX power supplies have a similar circuit. So it is necessary to study the circuit a little to adapt, the -12V line must have components similar to the +12V line, including wire thickness of filters, for that, whoever assembles it will need 2 power supplies, if they are reused from old computers

![img](https://raw.githubusercontent.com/rtek1000/DDS-915-DIY/main/Hardware/Doc/atxps2.png)

- - The pump needs 12V, which is already available in the ATX font
- - 5V voltage for the Arduino comes from the standby 5V line

- The OLED display can fade over time, so try to make it easy to replace
- - Be careful to buy a display with the same I2C address (0x3C)

- An alternative to the vacuum pump can be this model: [DC-555](https://www.aliexpress.com/item/32821282878.html) ([found in China](https://pt.aliexpress.com/w/wholesale-dc-555-pump.html)):

![img](https://raw.githubusercontent.com/rtek1000/DDS-915-DIY/main/Hardware/Doc/pump.png)

- - As this pump is not designed to generate a vacuum, it may be necessary to apply some reinforcement to the pump casing to prevent leaks

- Care must be taken due to the high working temperature, it is generally necessary to leave above 400°C / 450°C in this scheme so that the gun duct does not clog

- The most suitable working position is with the gun tip up, so that the solder residue is away from the heater duct

### Accessories

- The gun filter can be made with two layers of felt

(3M brand is very good quality, can be found as "Scotch-Brite Floor Cloth", in yellow color)

![img](https://upload.wikimedia.org/wikipedia/commons/1/1b/Colored_felt_cloth.jpg)

- The ZD-915 has a secondary filter, which includes a filtering element with a larger diameter, this filter can be made with a PET bottle nozzle, the screw cap makes it easy to change the filtering element, which can be a cotton ball, just to retain any impurities that pass through the gun's filter (which has two layers of felt)

![img](https://raw.githubusercontent.com/rtek1000/DDS-915-DIY/main/Hardware/Doc/filter_zd915.png)

- - Two water bottle spouts
- - Two aluminum rivets (used to shield sheet metal, POP Rivets). Inlet 4mm, outlet 6mm (outside diameter)
- - Two hex bolt nuts
- - Glue to seal the aluminum rivet to the bottle cap
- - A light spring can be used to keep the cotton ball pressed against the aluminum rivet

![img](https://raw.githubusercontent.com/rtek1000/DDS-915-DIY/main/Hardware/Doc/Filter2.png)

### Licence

#### Hardware:
Released under CERN OHL 1.2: https://ohwr.org/cernohl

#### Software:
This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
