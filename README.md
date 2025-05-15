# Software for parallel manipulator, named Stuwaldo

The final code for the control of parallel manipulator is under the project StuwaldoV3.

| Project                   | Description                                                                           |
|---------------------------|---------------------------------------------------------------------------------------|
| `Address_scanner_I2C`     | To scan for I2C address                                                               |
| `AnalogueDiscoveryTesting`    | Testing StuwaldoV3 with an Analogue Discovery. Sample rate: 100kHz. Contains bot images and csv data over for all samples                |
| `Calibration_platform`    | Calibration for the platform, calibrated values are read in the serial                |
| `LCD_Display`             | LCD display with states and cute messages                                             |
| `libraries`               | All libraries used thoughout this project                                             |
| `MATLAB_code`             | MATLAB code used for plotting data readings of actuators from the Arduino Due         |
| `one_leg_controll`        | Movement of one actuator at a time                                                    |
| `Platform`                | Initial start for the movement of platform                                            |
| `Platform_Anders`         | Platform code from another project that Anders provided                               |
| `plotter_example`         | Simple script showing how to plot values in serial plotter for Arduino IDE            |
| `Stuwaldo`                | First iteration of platform movement                                                  |
| `StuwaldoV2`              | Second version of the paltform, adding modes/states                                   |
| `StuwaldoV3`              | Final version of the paltform, soft start/stop, no big moment of inertia              |
| `Testing_buttons`         | Testing for buttons and three position switch, with states                            |
| `Testing_I2C_Display`     | Testing for the LCD 1602A LCD display with I2C backbone                               |
| `Testing_led_button`      | Testing for the led button on the Arduino Due with MOSFET                             |
| `watchDogExample`         | Example sketch for watchdog functionality for the Arduino Due                         |
| `read_serial.py`          | Reads from serial port and adds lines to csv file, used for plotting data in MATLAB   |
