# esp32_ros_ir_sensor

This is a little project for the ESP32 Dev Board based on [this repository](https://github.com/sachin0x18/rosserial_esp32).
It allows the user to connect multiple sensors like the [Sharp GP2Y0A21YK0F Infrared Proximity Sensor](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf)
to the ROS1 Melodic Ecosystem.
The contained source code is used to deliver distance values from a few connected sensors wirelessly to a ROS application.

## Getting Started

* install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#installation-step-by-step)
* clone repository
* unzip `rosserial_esp32.zip` in `$IDF_PATH/components` where $IDF_PATH is the installation path for ESP-IDF.
* load project
* connect an ESP32 Device to program
* make sure your user has the rights of the `dialout` group
* build, flash and monitor it
