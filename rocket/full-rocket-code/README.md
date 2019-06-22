# Requirements
- Teensy or Arduino Mega
- MPU6050 Accelerometer
- BMP180 Pressure Sensor
- uBlox NEO-6M GPS Breakout Board

For wiring see the pin schematic ![schematic](https://github.com/SamuelHeath/Avionics/raw/master/Schematic.png)

# Programming
- Use Arduino IDE (if using a Teensy you also need Teensyduino plugin to upload and compile for Teensy boards)
- Ideally you will need to have Serial1 for communicating if you wish to check the flight computer's output but you can easily add in debugging logging in the FlightLogger.cpp to view via the serial monitor the flight computer's output.
- Sensors are connected up using I2C and so its possible that you may need to check the device address using a I2C library that queries all possible addresses to make sure you're accessing the correct one.