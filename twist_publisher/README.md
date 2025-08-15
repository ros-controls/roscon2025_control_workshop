#### Sine Wave Twist Publisher

### Prerequisite
1. In VS Code use the `Open Folder` option to navigate to the `twist_publisher` directory
2. Allow PlatformIO to download `zenoh-pico` into the folder `.pio/libdeps/esp32-s3-devkitm-1/zenoh-pico` (this step happens automatically after you open the folder with the PlatformIO extension enabled)
3. Clone additional [dependencies](platformio.ini#L35) `Pico-ROS` and `Micro-CDR` into the directory `.pio/libdeps/esp32-s3-devkitm-1/`
4. Install [python virtual env](https://docs.platformio.org/en/latest/faq/install-python.html) `sudo apt install python3-venv`.
5. Setup the 99-platformio-udev.rules as per the [instructions](https://docs.platformio.org/en/latest/core/installation/udev-rules.html#platformio-udev-rules)
`curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules`
`sudo service udev restart`
6. Update [src/main.cpp](src/main.cpp#L10-L15) to match your host's 
- `ROUTER_ADDRESS` (see note below on running the router on the host)
- WiFi `SSID`
- WiFi `PASS`

## Running the Example

1. Start and open an interactive shell to the Workshop container
```bash
docker exec -it ros2_control_roscon25 bash
```

2. Start the Zenoh router and note the wireless IP address it is running on
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

You should see something like this in the output
```
2025-08-13T14:37:00.271196Z  INFO ThreadId(02) zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/192.168.9.241:7447
```

3.  Follow the `Prerequisite` above and build the project with `Ctrl+Alt+B` then upload the code to the device with `Ctrl+Alt+U`

4. Once the program starts on the ESP32 the 
- LED will start blinking Blue to indicate it is connecting to the WiFi
- Serial message should be printed though PlatformIO's Serial Monitor
- LED will start blinking Green after it has connected to to the Zenoh host and is publishing messages

5. On the host open a second interactive terminal to the Workshop container and open Plogjuggler
```bash
docker exec -it ros2_control_roscon25 bash
ros2 run plotjuggler plotjuggler
```

6. Select `Start` in the Plugjuggler UI and select the `picoros/cmd_vel` topic to listen to and add the `Linear.X` and `Angular.Z` data to the plot

<img src="../docs/picoros_cmd_vel_example_plot.png">
