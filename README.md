Author: Yasir Aslam Shah IOT Embedded Firmware Blue Gecko Bluetooth SDK 2.7 SPring 2018

Wireless-Vehicle-Dash-Board
WIreless Vehicle Dash Board Client Module 
This project contains a Bluetooth low energy Client based wireless vehicle dash board. The server contains APDS 9031 light sensor,si7021 Humidity Sensor,BMA280 Accelerometer sensor and a GLCD, while as the client contains a GLCD to display data from server.

 The client server pairing is authorized using MITM(Man in the Middle Protocol)

 Persistent memory,I2C,SPI,BLE, Scheduler and Bluetooth Low Energy Service is also implemented as a part of project.

 Sensor data is represented as a state on LCD. Each sensor has two states for the client. 
 BMA280 is represented as “Accelerating” and ”Non-Accelerating” 
 Light Sensor is represented as “Day Light” and “Night” and is used as a service to the client 
 Humidity sensor is represented as “Humid” and ”Dry” and is used in Persistent Memory Application
