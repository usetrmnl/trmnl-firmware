BQ27427 Battery Fuel Gauge Arduino Library
==========================================

The Texas Instruments [BQ27427](https://www.ti.com/product/BQ27427) battery fuel gauge is a single-cell gauge that requires minimal user configuration and system microcontroller firmware development, leading to quick system bring-up -- it measures your battery's voltage to estimate its charge percentage and remaining capacity. The chip contains an internal current-sensing resistor, which allows it to measure current and power!

This Arduino library abstracts away all of the low-level I2C communication, so you can easily initialize the fuel gauge then read voltage, state-of-charge, current, power, and capacity. It also implements all of the chip's low-battery, and SoC-change alerts on the GPOUT pin.

Thanks to:

* [Sparkfun Electronics, Inc.](https://github.com/sparkfun) for the [BQ27441-G1A library](https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library) which this library is heavily based on.

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

Documentation
--------------

* **[BQ27427 Datasheet](https://www.ti.com/lit/ds/symlink/bq27427.pdf)** - The full datasheet for the TI BQ27427 integrated ciruit.
* **[BQ27427 Technical Reference Manual](https://www.ti.com/lit/ug/sluucd5/sluucd5.pdf)** - The Technical Reference Manual for the TI BQ27427.


Version History
---------------

* [1.0.4](https://github.com/edreanernst/BQ27427_Arduino_Library/releases/tag/1.0.4) - Add reading of current measurement polarity.
* [1.0.3](https://github.com/edreanernst/BQ27427_Arduino_Library/releases/tag/1.0.3) - Bugfix: Correct DeviceID.
* [1.0.2](https://github.com/edreanernst/BQ27427_Arduino_Library/releases/tag/1.0.2) - Add ability to interact with discharge current threshold and taper voltage settings.
* [1.0.1](https://github.com/edreanernst/BQ27427_Arduino_Library/releases/tag/1.0.1) - Add ability to change current sensing polarity.
* [1.0.0](https://github.com/edreanernst/BQ27427_Arduino_Library/releases/tag/1.0.0) - Initial release of the BQ27427 Arduino Library.

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

Distributed as-is; no warranty is given.
