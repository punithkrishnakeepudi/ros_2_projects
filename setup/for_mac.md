# ROS2 Jazzy on macOS Using Docker
### Complete Beginner Guide — From Installation to Your First ROS2 Node

---

## Why Use Docker?

ROS2 Jazzy is officially supported on Ubuntu 24.04. While it's possible to install ROS2 directly on macOS, the Docker approach is:

- Easier to install
- Consistent with official ROS tutorials
- Free of dependency headaches
- Works the same on Intel and Apple Silicon Macs
- Widely used by ROS developers on macOS

This guide uses **only Docker**.

---

## System Requirements

- macOS Ventura, Sonoma, or newer
- At least 15 GB free disk space
- Internet connection
- Terminal application

---

## Step 1: Install Docker Desktop

Download Docker Desktop for Mac: [docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop/)

Install it like any normal macOS app, then:

1. Open Docker Desktop
2. Wait until Docker starts successfully
3. Verify Docker is running

Check the version:

```bash
docker --version
```

Expected output: `Docker version xx.x.x`

Test Docker:

```bash
docker run hello-world
```

You should see `Hello from Docker!`

---

## Step 2: Download the ROS2 Jazzy Docker Image

```bash
docker pull ros:jazzy
```

This may take several minutes.

Verify:

```bash
docker images
```

You should see `ros    jazzy` in the list.

---

## Step 3: Create a Permanent ROS Workspace Folder on Your Mac

```bash
mkdir -p ~/ros2_ws
```

Verify:

```bash
ls ~
```

You should see `ros2_ws` listed.

---

## Step 4: Start a ROS2 Docker Container

```bash
docker run -it \
  --name ros2_jazzy \
  -v ~/ros2_ws:/root/ros2_ws \
  ros:jazzy
```

**Explanation:**

| Flag | Meaning |
|------|---------|
| `-it` | Interactive terminal |
| `--name` | Container name |
| `-v` | Mounts your Mac folder into the container |

Mapping: Mac `~/ros2_ws`  ↔  Docker `/root/ros2_ws`

Anything created inside Docker's `/root/ros2_ws` is automatically saved on your Mac.

---

## Step 5: Verify ROS2 Installation

Inside Docker:

```bash
source /opt/ros/jazzy/setup.bash
```

Check ROS:

```bash
ros2 --help
```

You should see the list of ROS2 commands.

Verify version:

```bash
ros2 doctor
```

---

## Step 6: Create a ROS2 Workspace

Inside Docker:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

Verify:

```bash
ls
```

Expected: `src`

---

## Step 7: Build the Empty Workspace

```bash
colcon build
```

After the build:

```bash
ls
```

Expected: `build  install  log  src`

ROS2 generates these folders automatically.

---

## Step 8: Source the Workspace

After **every** build, source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Verify:

```bash
echo $ROS_DISTRO
```

Expected: `jazzy`

---

## Step 9: Automatically Source ROS2

Open bash configuration:

```bash
nano ~/.bashrc
```

Add this line:

```bash
source /opt/ros/jazzy/setup.bash
```

Save and exit nano: `CTRL + O`, `ENTER`, `CTRL + X`

Reload:

```bash
source ~/.bashrc
```

Now every new terminal inside Docker loads ROS automatically.

---

## Step 10: Create Your First ROS Package

Move to `src`:

```bash
cd ~/ros2_ws/src
```

Create the package:

```bash
ros2 pkg create \
  --build-type ament_python \
  my_robot
```

You should see `creating package...`

---

## Step 11: Understand the Package Structure

```bash
cd my_robot
```

Structure:

```
my_robot/
├── my_robot/
│   └── __init__.py
├── resource/
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

| File/Folder | Purpose |
|---|---|
| `my_robot/` | Python code |
| `package.xml` | Dependencies |
| `setup.py` | Registers nodes |
| `resource/` | ROS metadata |
| `test/` | Unit tests |

---

## Step 12: Create Your First Node

Move into the Python package folder:

```bash
cd ~/ros2_ws/src/my_robot/my_robot
```

Create the node file:

```bash
touch hello_node.py
chmod +x hello_node.py
```

Open it:

```bash
nano hello_node.py
```

Paste this code:

```python
import rclpy
from rclpy.node import Node

class HelloNode(Node):

    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello ROS2 on Mac!')

def main(args=None):

    rclpy.init(args=args)

    node = HelloNode()

    rclpy.spin_once(node, timeout_sec=1)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save and exit.

---

## Step 13: Register the Node

Open:

```bash
nano ~/ros2_ws/src/my_robot/setup.py
```

Find:

```python
entry_points={
    'console_scripts': [
    ],
},
```

Replace with:

```python
entry_points={
    'console_scripts': [
        'hello_node = my_robot.hello_node:main',
    ],
},
```

Save.

---

## Step 14: Build the Package

Return to the workspace:

```bash
cd ~/ros2_ws
```

Build:

```bash
colcon build --packages-select my_robot
```

Wait until the build completes.

---

## Step 15: Source the Workspace Again

```bash
source install/setup.bash
```

This step is required after **every** build.

---

## Step 16: Verify ROS Sees the Package

```bash
ros2 pkg list | grep my_robot
```

Expected: `my_robot`

---

## Step 17: Run Your First ROS Node

```bash
ros2 run my_robot hello_node
```

Expected output:

```
[INFO] [hello_node]: Hello ROS2 on Mac!
```

🎉 Congratulations! Your first ROS2 node is running.

---

## Understanding the ROS2 Workflow

Every time you create or modify code, repeat this cycle:

1. **Edit** code
2. **Build** package
   ```bash
   colcon build --packages-select my_robot
   ```
3. **Source** workspace
   ```bash
   source install/setup.bash
   ```
4. **Run** node
   ```bash
   ros2 run my_robot hello_node
   ```

---

## Useful ROS2 Commands

List packages:
```bash
ros2 pkg list
```

List running nodes:
```bash
ros2 node list
```

List topics:
```bash
ros2 topic list
```

List services:
```bash
ros2 service list
```

Show topic data:
```bash
ros2 topic echo /topic_name
```

Show interface definition:
```bash
ros2 interface show std_msgs/msg/String
```

Check ROS environment:
```bash
ros2 doctor
```

---

## Working with Docker Later

Stop the container:
```bash
exit
```
or
```bash
docker stop ros2_jazzy
```

Start the container again:
```bash
docker start ros2_jazzy
```

Open a terminal inside the container:
```bash
docker exec -it ros2_jazzy bash
```

Reload ROS:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Continue working exactly where you left off.

---

## Quick Start for a New Terminal Session

Whenever you want to run the ROS2 Docker container in a **new terminal**, use these steps:

```bash
docker start ros2_jazzy
docker exec -it ros2_jazzy bash
source /opt/ros/jazzy/setup.bash
```

Then, to test a working publisher/subscriber pair, open **two terminals**:

**Terminal 1:**
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2:**
```bash
ros2 run demo_nodes_cpp listener
```

---

