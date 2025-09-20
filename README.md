# 🧭🤖 magLaserBot

<div style="display: inline_block">
<img align="center" src="/image/LaserMagnetometer.jpg" alt="Foto" width="40%">
<img align="center" src="/image/motors.jpg" alt="Foto" width="40%">
<img align="center" src="/image/ESP32.jpg" alt="Foto" width="30%">
<img align="center" src="/image/magLaserBot.gif" alt="Demonstração" width="30%">
</div>

**magLaserBot** is an autonomous mapping robot built on an ESP32 microcontroller. It combines magnetometer-based navigation with obstacle avoidance and real-time occupancy mapping using a time-of-flight distance sensor. The system displays the generated map on an OLED screen and logs data via Serial.

## 📦 Hardware Components

| Component            | Description                                      |
|---------------------|--------------------------------------------------|
| ESP32               | Main microcontroller                             |
| VL53L0X             | Time-of-flight distance sensor                   |
| QMC5883L Compass    | Magnetometer for heading detection               |
| SSD1306 OLED (128x64)| Display for visualizing map and robot position  |
| N20 Motors (x2)     | Geared DC motors with encoders                   |
| L298N/TB6612FNG     | Motor driver                                     |
| I2C Bus             | Communication protocol between sensors           |

## 🧠 How It Works

1. **Initialization**
   - Scans the I2C bus for connected devices
   - Initializes the OLED display, VL53L0X sensor, and magnetometer
   - Calibrates motor encoders and driver

2. **Navigation & Mapping**
   - Reads magnetometer data to determine azimuth (heading)
   - Applies magnetic declination correction
   - Reads distance from VL53L0X sensor for obstacle detection
   - Updates occupancy grid with detected obstacles
   - Uses encoder data for precise odometry and position tracking

3. **Obstacle Avoidance**
   - Implements reactive obstacle avoidance behavior
   - Makes random turns when obstacles are detected
   - Continues mapping while navigating

4. **Visualization**
   - Displays occupancy grid on OLED display
   - Shows robot position and orientation in real-time
   - Provides HUD with position and sensor data

## 🗺️ Occupancy Grid Mapping

The robot builds a 50×50 occupancy grid where:
- `0` = Free space
- `1` = Occupied (obstacle)
- `2` = Unknown

Each cell represents 20mm × 20mm of physical space.

## ⚙️ Requirements

- [PlatformIO](https://platformio.org/) installed in Visual Studio Code
- Required libraries:
  - `Adafruit_GFX`
  - `Adafruit_SSD1306`
  - `Adafruit_VL53L0X`
  - `QMC5883LCompass`

## 🚀 Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/magLaserBot.git
   cd magLaserBot
   ```

2. Open the project in VS Code with PlatformIO

3. Connect your ESP32 via USB

4. Build and upload the firmware

5. Open the Serial Monitor at 115200 baud to view live logs and mapping data

## 🖥️ OLED Display

The OLED screen shows:
- Occupancy grid map with obstacles
- Robot position (dot) and orientation (line)
- Real-time HUD with coordinates (X, Y) and heading (θ)

## 🤖 Robot Specifications

- Wheel diameter: 32mm
- Wheel base: 90mm
- Encoder resolution: 330 pulses per revolution
- Maximum mapping range: 1000mm × 1000mm
- Obstacle detection range: up to 2000mm

## 🧪 Calibration & Tuning

- Magnetometer calibration can be adjusted using:
```cpp
compass.setCalibration(mx_min, mx_max, my_min, my_max);
```

- Adjust robot parameters in code:
```cpp
const float WHEEL_DIAMETER = 32.0;    // mm
const float WHEEL_BASE = 90.0;        // mm
const int PULSES_PER_REVOLUTION = 330;
```

- Set `magnetic_declination` according to your geographic location

## 📊 Serial Output Format

The robot outputs both mapping data and navigation status:
```
Pos: 125.5, 87.2, 45.0° | Dist: 320 mm
Grid: X:25, Y:17 | Value: 1
```

## 🔧 Wiring Guide

| ESP32 Pin | Component       | Connection      |
|-----------|-----------------|-----------------|
| GPIO 21   | I2C             | SDA             |
| GPIO 22   | I2C             | SCL             |
| GPIO 4    | Motor Driver    | STBY            |
| GPIO 14   | Motor Driver    | AIN1            |
| GPIO 27   | Motor Driver    | AIN2            |
| GPIO 12   | Motor Driver    | PWMA            |
| GPIO 32   | Motor Driver    | BIN1            |
| GPIO 33   | Motor Driver    | BIN2            |
| GPIO 13   | Motor Driver    | PWMB            |
| GPIO 25   | Encoder Left    | Channel A       |
| GPIO 26   | Encoder Left    | Channel B       |
| GPIO 34   | Encoder Right   | Channel A       |
| GPIO 35   | Encoder Right   | Channel B       |

## 🗺️ Future Improvements

- Add support for SLAM algorithms
- Implement path planning and waypoint navigation
- Add wireless communication for remote monitoring
- Integrate with ROS for advanced robotics applications
- Add SD card support for data logging
- Implement automatic calibration routines

## 🛡️ Safety Features

- Automatic stop when obstacles are too close
- Emergency stop capability via serial command
- Low battery detection and warning system
- Motor current monitoring

## 🤝 Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

**Note**: This project is for educational and research purposes. Always exercise caution when working with moving robots and ensure proper safety measures are in place.