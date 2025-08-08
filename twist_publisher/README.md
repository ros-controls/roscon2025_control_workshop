# Twist Publisher Example

This example depends on PlatformIO to manage the embedded system libraries and devices and assumes you are working with an ESP32 `esp32-s3-devkitm-1` development board.

Once the [PlatformIO](https://platformio.org/) dependency extension is installed in VS Code, follow the steps below to build and program the device:

1. In VS Code use the `Open Folder` option to navigate to the `twist_publisher` directory
1. Allow Platform to download `zenoh-pico` into the folder `.pio/libdeps/esp32-s3-devkitm-1/zenoh-pico` (this step happens automatically after you open the folder with the PlatformIO extension enabled)
1. Clone additional [dependencies](../twist_publisher/platformio.ini#L32-L33) `Pico-ROS` and `Micro-CDR` into the directory `.pio/libdeps/esp32-s3-devkitm-1/` 
1. Build the project with `Ctrl+Alt+B`
1. If you have the device connected you can flash it to test the twist publisher.

