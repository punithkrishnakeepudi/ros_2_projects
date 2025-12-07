# ROS2 + NodeMCU External LED Blink — Simple Guide (Markdown Version)

This guide shows ONLY how to control **an external LED on D2 (GPIO4)** of a NodeMCU using **ROS2**.

---

## 1. What you need
- NodeMCU (ESP8266)
- USB data cable
- External LED (any color)
- 220Ω resistor
- Ubuntu laptop with ROS2 installed
- Arduino IDE or PlatformIO

---

## 2. Wiring the external LED (D2 → GPIO4)
Connect it exactly like this:
```
NodeMCU D2 (GPIO4) → 220Ω Resistor → LED Anode (long leg)
LED Cathode (short leg) → GND
```
**Logic:**
- `HIGH` → LED ON
- `LOW` → LED OFF

---

## 3. Arduino Code (external LED serial listener)
Upload this code to your NodeMCU.

**File: `nodemcu_external_led.ino`**
```cpp
const int LED_EXT = 4; // D2 -> GPIO4
unsigned long blink_period = 500UL;
bool blinking = false;
unsigned long last_toggle = 0;
bool led_state = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_EXT, OUTPUT);
  digitalWrite(LED_EXT, LOW);
  Serial.println("Ready - External LED on D2");
}

void loop() {
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "ON") {
      blinking = false;
      digitalWrite(LED_EXT, HIGH);
      Serial.println("OK:ON");
    }
    else if (cmd == "OFF") {
      blinking = false;
      digitalWrite(LED_EXT, LOW);
      Serial.println("OK:OFF");
    }
    else if (cmd.startsWith("BLINK")) {
      int spaceIndex = cmd.indexOf(' ');
      unsigned long speed = (spaceIndex > 0) ? cmd.substring(spaceIndex + 1).toInt() : 500;
      if (speed < 50) speed = 50;

      blink_period = speed;
      blinking = true;
      last_toggle = millis();
      led_state = false;
      Serial.printf("OK:BLINK %lu\n", blink_period);
    }
    else if (cmd == "STOP") {
      blinking = false;
      Serial.println("OK:STOP");
    }
  }

  if (blinking) {
    if (millis() - last_toggle >= blink_period / 2) {
      last_toggle = millis();
      led_state = !led_state;
      digitalWrite(LED_EXT, led_state ? HIGH : LOW);
    }
  }
}
```

---

## 4. Create ROS2 package (bridge)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ROSDISTRO=$(ls /opt/ros | head -n1)
source /opt/ros/$ROSDISTRO/setup.bash
ros2 pkg create --build-type ament_python led_serial_bridge --dependencies rclpy std_msgs
```

---

## 5. ROS2 Python node
Create:
```
~/ros2_ws/src/led_serial_bridge/led_serial_bridge/ros2_serial_led.py
```

Paste:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, os, time

SERIAL_PORT = os.environ.get("NODEMCU_PORT", "/dev/ttyUSB0")
BAUDRATE = 115200

class LedSerialBridge(Node):
    def __init__(self):
        super().__init__('led_serial_bridge')
        self.get_logger().info(f'Opening serial port {SERIAL_PORT} @ {BAUDRATE}')
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(0.1)
        self.sub = self.create_subscription(String, 'led_cmd', self.on_cmd, 10)
        self.get_logger().info('Listening on /led_cmd')

    def on_cmd(self, msg):
        cmd = msg.data.strip().upper() + "\n"
        self.ser.write(cmd.encode("utf-8"))
        self.get_logger().info(f"Sent -> {cmd.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = LedSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. Setup entry point
Edit:
```
~/ros2_ws/src/led_serial_bridge/setup.cfg
```
Add:
```ini
[options.entry_points]
console_scripts =
    ros2_serial_led = led_serial_bridge.ros2_serial_led:main
```

Create `setup.py`:
```python
from setuptools import setup
setup(
    name='led_serial_bridge',
    version='0.0.0',
    packages=['led_serial_bridge'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': ['ros2_serial_led = led_serial_bridge.ros2_serial_led:main'],
    },
)
```

---

## 7. Build & Run
```bash
cd ~/ros2_ws
source /opt/ros/$ROSDISTRO/setup.bash
python3 -m pip install --user pyserial setuptools
colcon build --packages-select led_serial_bridge
. install/setup.bash

export NODEMCU_PORT=/dev/ttyUSB0
ros2 run led_serial_bridge ros2_serial_led
```

---

## 8. ROS2 Commands
```bash
ros2 topic pub /led_cmd std_msgs/String "data: 'ON'"
ros2 topic pub /led_cmd std_msgs/String "data: 'OFF'"
ros2 topic pub /led_cmd std_msgs/String "data: 'BLINK 200'"
ros2 topic pub /led_cmd std_msgs/String "data: 'STOP'"
```

---
