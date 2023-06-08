# DDS-915-DIY
Digital desoldering station (ZD-915 based)
- Using Arduino NANO board

(Working in progress)

### References

Watch these videos to understand how it works:

- [ZD-915 Desoldering Station review](https://www.youtube.com/watch?v=_Ar05rKqoEI)

- [ZD-915 Desoldering gun upgrade - overvoltage fix](https://www.youtube.com/watch?v=MPcmVaqe08Y)

Additional Information: [ZD-915 digital desoldering station](https://eleshop.eu/desolderingstation-zd-915.html)

### Software

- The Arduino sketch takes up almost all of the ATmega328's capacity.

- PID type temperature control is used

- The biggest consumption of resources is the OLED display (0.96 inches) on the I2C bus.

### Hardware

Schematic:

![img](https://raw.githubusercontent.com/rtek1000/DDS-915-DIY/main/Hardware/Doc/DDS-915-DIY_Kicad.png)

- The power supply was adapted from an ATX computer power supply
- - The gun uses 24V, stronger diodes are needed to get -12V high power
- - - The +12V with the -12V form the 24V for the gun
- - - It is also necessary to adapt the filters
- - The pump needs 12V, which is already available in the ATX font
- - 5V voltage for the Arduino comes from the standby 5V line

- The OLED display can fade over time, so try to make it easy to replace
- - Be careful to buy a display with the same I2C address (0x3C)

- A good alternative to the vacuum pump can be this model: [DC-555](https://www.aliexpress.com/item/32821282878.html) ([found in China](https://pt.aliexpress.com/w/wholesale-dc-555-pump.html)):

![img](https://raw.githubusercontent.com/rtek1000/DDS-915-DIY/main/Hardware/Doc/pump.png)


### Accessories

- The gun filter can be made with two layers of felt

(3M brand is very good quality, can be found as "Scotch-Brite Floor Cloth", in yellow color)

![img](https://upload.wikimedia.org/wikipedia/commons/1/1b/Colored_felt_cloth.jpg)

### Licence

#### Hardware:
Released under CERN OHL 1.2: https://ohwr.org/cernohl

#### Software:
This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
