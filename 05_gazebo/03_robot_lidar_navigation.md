# Build a Robot with LiDAR — ROS 2 Simulation

This is the final beginner project.

You will give your robot eyes, and then a tiny brain.

At the end, your robot will drive around obstacles **by itself**.

You need everything from the previous two documents:

* [01 — Gazebo Installation](01_gazebo_installation.md)
* [02 — Gazebo + ROS 2 + RViz](02_gazebo_ros2_rviz.md)

> [!NOTE]
> Reminder: run `source /opt/ros/jazzy/setup.bash` in **every** new terminal
> (unless it is already in your `~/.bashrc`).

---

# 1. What are we building?

```text
             WALL
      █████████████████

        ↖  ↑ ↑ ↑ ↗
         \ | | | /
          \| | |/
           \| |/
             🤖
            LiDAR
```

The robot will use LiDAR to measure how far objects are.

Then a small Python program will read those measurements and decide:

* Path clear? → drive forward.
* Obstacle ahead? → turn toward open space.

That is a real autonomous robot. A simple one, but real.

---

# 2. What is LiDAR?

LiDAR sends laser beams and measures distance.

The name means **Li**ght **D**etection **a**nd **R**anging.

It works like this: send out a laser beam, wait for it to bounce back, measure how long it took. Light travels at a known speed, so the time tells you the distance.

```text
Robot
  │
  ├── laser → wall          "2.1 meters"
  │
  ├── laser → object        "0.8 meters"
  │
  └── laser → open space    "nothing (too far)"
```

A LiDAR does not send one beam. It spins and sends **hundreds of beams in all directions**, many times per second.

So the robot receives many distance measurements — a full circle of "how far is the nearest thing in that direction?"

Robot vacuum cleaners have exactly this sensor (the little spinning tower on top).

---

# 3. LiDAR in Gazebo

We are not putting a real LiDAR on the computer.

Gazebo creates a virtual LiDAR.

```text
REAL ROBOT

Physical LiDAR
      ↓
Real data


SIMULATION

Virtual LiDAR
      ↓
Simulated data
```

Gazebo knows where every wall and box is (it built them!). So it can calculate exactly what a laser beam would hit. It then produces sensor data in the **same format** as a real LiDAR.

This is the superpower of simulation: the Python program we write today would work with a real LiDAR too, without changes.

---

# 4. Add LiDAR to the robot

We will create a new robot file. It is the same robot as document 02, plus:

* a new link for the laser
* a new joint holding the laser
* the laser sensor settings

Create the file:

```bash
nano ~/ros2_ws/src/simulation/my_robot_lidar.urdf
```

Paste **all** of this (yes, it is long — you already know most of it from document 02):

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- ================== BODY ================== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.0417" ixy="0" ixz="0"
               iyy="0.0708" iyz="0" izz="0.1042"/>
    </inertial>
  </link>

  <!-- ================== LEFT WHEEL ================== -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00135" ixy="0" ixz="0"
               iyy="0.0025" iyz="0" izz="0.00135"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.1 0.175 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- ================== RIGHT WHEEL ================== -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00135" ixy="0" ixz="0"
               iyy="0.0025" iyz="0" izz="0.00135"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.1 -0.175 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- ================== CASTER ================== -->
  <link name="caster">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.25"/>
      <inertia ixx="0.00025" ixy="0" ixz="0"
               iyy="0.00025" iyz="0" izz="0.00025"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="0.15 0 -0.05" rpy="0 0 0"/>
  </joint>

  <gazebo reference="caster">
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>

  <!-- ================== LIDAR (NEW!) ================== -->
  <!-- A small cylinder sitting on top of the body -->
  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.9 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0 0 0.07" rpy="0 0 0"/>
  </joint>

  <!-- The LiDAR sensor settings (only Gazebo reads this part) -->
  <gazebo reference="laser_link">
    <sensor name="lidar" type="gpu_lidar">
      <topic>scan</topic>
      <update_rate>10</update_rate>
      <gz_frame_id>laser_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.2</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
      <always_on>true</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>

  <!-- ================== GAZEBO PLUGINS ================== -->
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.35</wheel_separation>
      <wheel_radius>0.1</wheel_radius>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>

    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
    </plugin>
  </gazebo>

</robot>
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

The important new sections:

* **`laser_link`** — the physical laser device: a small red cylinder.
* **`laser_joint`** — glues the laser on top of the body.
* **`<sensor>`** — tells Gazebo: "this link is not just a cylinder — it is a LiDAR, and here are its settings."

The rest of the file is identical to document 02.

---

# 5. LiDAR link

Why does the LiDAR need its own link?

```text
Robot
 │
 └── base_link
       │
       └── laser_link
```

Two reasons:

1. **Position.** A distance measurement means "2 meters *from the laser*". ROS 2 must know exactly where the laser sits on the robot. A separate link gives it an exact position.
2. **Honesty.** On a real robot, the LiDAR is a real device mounted at a real spot. Our simulation copies reality: one device = one link.

Our laser sits at `xyz="0 0 0.07"` — at the center of the body, 7cm above the body's middle. High enough that the laser beams fly over the robot's own body instead of hitting it.

---

# 6. LiDAR joint

```text
base_link
    │
    │ joint
    ▼
 laser_link
```

The joint connects the LiDAR to the robot.

We used a **fixed** joint. Fixed means glued: the laser never moves relative to the body. (Wheels needed `continuous` joints because they spin. The laser just sits there.)

Because of this joint, when the robot drives forward, the laser automatically comes along — and TF always knows where it is.

---

# 7. LiDAR settings

Look at the `<sensor>` block again. Here is what each setting means:

```text
Minimum range = closest distance the sensor can measure

Maximum range = farthest distance the sensor can measure
```

Our settings, one by one:

* **`<min>0.2</min>`** — anything closer than 20cm is invisible. Real LiDARs have this limit too (the beam needs some distance to work).
* **`<max>12.0</max>`** — anything farther than 12 meters is invisible. The sensor reports "nothing there".
* **`<samples>360</samples>`** — the number of rays. We use 360 rays in a full circle: **one ray per degree**. Remember this — we will use it in our Python code.
* **`<min_angle>0.0</min_angle>` and `<max_angle>6.28</max_angle>`** — the field of view. From 0 radians to 6.28 radians (= 360 degrees = a full circle). Ray number 0 points **forward**.
* **`<update_rate>10</update_rate>`** — measure 10 times per second.

Picture it:

```text
            ray 90 (left)
               ↑
        ↖      │      ↗
           \   │   /
ray 180 ←───── 🤖 ─────→ ray 0 (front)
           /   │   \
        ↙      │      ↘
               ↓
            ray 270 (right)
```

Ray 0 = front. Ray 90 = left. Ray 180 = back. Ray 270 = right.

(In ROS 2, angles turn counter-clockwise, and "left" is the positive Y direction. That is why 90 is left, not right.)

---

# 8. Connect LiDAR to ROS 2

The data path looks like this:

```text
LiDAR
  │
  ▼
Gazebo
  │
  ▼
ROS 2
  │
  ▼
/scan
```

`/scan` is the standard ROS 2 topic name for laser data. Almost every laser-equipped robot in the ROS world publishes on `/scan`.

## One new thing: the world needs a sensor engine

Here is a Gazebo rule you have not met yet:

**Sensors only work if the world loads the "Sensors" plugin.**

Our `first_world.sdf` did not have it. Gazebo quietly loaded a set of default helpers for us (physics, communication). But the sensors helper is not in the defaults — and once we list plugins ourselves, we must list ALL the helpers we need.

So we make a new test world. Create it:

```bash
nano ~/ros2_ws/src/simulation/lidar_test_world.sdf
```

Paste:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="lidar_test_world">

    <!-- WORLD HELPERS (plugins). Sensors is the new one! -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- One wall, 2 meters in front of the robot -->
    <model name="test_wall">
      <static>true</static>
      <pose>2 0 0.25 0 0 0</pose>
      <link name="wall_link">
        <collision name="wall_collision">
          <geometry>
            <box><size>0.2 4 0.5</size></box>
          </geometry>
        </collision>
        <visual name="wall_visual">
          <geometry>
            <box><size>0.2 4 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.3 0.3 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

Save and exit.

The four plugin lines, in simple words:

```text
Physics          →  gravity, wheels, crashing
UserCommands     →  lets us spawn the robot into the world
SceneBroadcaster →  lets the Gazebo window show the world
Sensors          →  the NEW one: makes cameras and LiDARs work
```

## Start everything

**Terminal 1** — world:

```bash
gz sim -r ~/ros2_ws/src/simulation/lidar_test_world.sdf
```

**Terminal 2** — spawn the new robot:

```bash
ros2 run ros_gz_sim create -file ~/ros2_ws/src/simulation/my_robot_lidar.urdf -name my_robot -x 0 -y 0 -z 0.2
```

Look at Gazebo — you should see blue laser rays fanning out of the robot! (That is the `<visualize>true</visualize>` setting.) The rays stop at the red wall.

**Terminal 3** — the bridge, now with one extra line for `/scan`:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist' \
  '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry' \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V' \
  '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

The new line translates laser data from Gazebo into the ROS 2 topic `/scan`. (The `[` means one-way: Gazebo → ROS 2. Sensor data only flows out of the sensor.)

---

# 9. Check /scan

**Terminal 4** — list the topics:

```bash
ros2 topic list
```

You should now see `/scan` in the list.

Look inside it (one message only — a full scan is big):

```bash
ros2 topic echo /scan --once
```

You will see something like:

```text
header:
  stamp:
    sec: 15
  frame_id: laser_link
angle_min: 0.0
angle_max: 6.28
range_min: 0.2
range_max: 12.0
ranges:
- 1.9
- 1.91
- 1.93
- 2.05
- .inf
- .inf
- 3.2
...
```

Do not try to memorize this. The important part is the long `ranges:` list.

**The numbers are distances measured around the robot.**

* `1.9` → this ray hit something 1.9 meters away (our wall!)
* `.inf` → "infinity" — this ray hit nothing within 12 meters (open space)

360 numbers, one per degree, ten times per second. That is everything a LiDAR says.

---

# 10. LaserScan

The message type on `/scan` is called **LaserScan**. You only need five of its fields:

```text
ranges
angle_min
angle_max
range_min
range_max
```

* **`ranges`** — the list of distances. `ranges[0]` is the first ray, `ranges[1]` the second, and so on.
* **`angle_min`** — the direction of the FIRST ray (for us: 0.0 = straight ahead).
* **`angle_max`** — the direction of the LAST ray (for us: 6.28 = all the way around).
* **`range_min`** — closest valid distance (0.2m). Smaller values are garbage.
* **`range_max`** — farthest valid distance (12m). Larger values (or `.inf`) mean "nothing there".

```text
angle_min                 angle_max
     ↘                       ↙
       \                   /
        \       🤖        /
         \               /
          \_____________/
              LiDAR
```

For our robot the two angles meet behind a full circle, so the picture is a complete ring of measurements.

The rule to remember for our LiDAR:

```text
ranges[0]    →  distance straight AHEAD
ranges[90]   →  distance to the LEFT
ranges[180]  →  distance BEHIND
ranges[270]  →  distance to the RIGHT
```

---

# 11. See LiDAR in RViz

Keep Terminals 1–3 running (world, robot, bridge).

**Terminal 4** — robot shape publisher (same as document 02, but with the new URDF):

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat ~/ros2_ws/src/simulation/my_robot_lidar.urdf)" \
  -p use_sim_time:=true
```

**Terminal 5** — RViz:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

Set up RViz:

1. **Fixed Frame** → `odom`
2. **Add** → **RobotModel** → set **Description Topic** to `/robot_description`
3. **Add** → **TF**
4. **Add** → **LaserScan**

Now set the LaserScan topic:

```text
LaserScan (left panel)
   ↓
Topic
   ↓
/scan
```

Tip: also open the LaserScan settings and set **Size (m)** to `0.05` — bigger dots are easier to see.

### What should you see?

A curve of red/white dots in front of the robot — exactly where the wall is!

```text
Gazebo
   │
   │ LiDAR
   ▼
 /scan
   │
   ▼
 RViz
   │
   ▼
Laser points
```

This is a big moment. In document 02, RViz showed an empty world around the robot — the robot was blind. Now the robot **perceives** the wall, and RViz draws what it perceives.

Remember: RViz is not showing the wall. It is showing the robot's *measurements* of the wall.

---

# 12. Activity — Find an obstacle

The wall is already in front of the robot:

```text
Robot → Wall
```

### Do this

1. In Terminal 6, watch the front distance update live:
   ```bash
   ros2 topic echo /scan --field ranges[0]
   ```
   You should see roughly `1.9` printed again and again (the wall is 2 meters away, minus half the wall's thickness).
2. Now go to the Gazebo window. Click the wall to select it. Use the **move tool** (the arrows icon in the top toolbar) and drag the wall closer to the robot.

### What should happen?

Do the LiDAR values change?

They should! Drag the wall closer → the number drops. Drag it away → the number grows.

You are watching a sensor measure the world in real time.

### Challenge

Drag the wall closer than 20cm to the laser. What happens to the value?

(It becomes `.inf` or garbage — closer than `range_min`, the sensor cannot see. Every sensor has limits. Good robot programs respect them.)

---

# 13. Read LiDAR using Python

Watching numbers is nice. Using them in a program is power.

Let us write the smallest possible LiDAR program: read the front distance and print it.

Create the file:

```bash
nano ~/ros2_ws/src/simulation/read_lidar.py
```

Paste:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarReader(Node):

    def __init__(self):
        super().__init__('lidar_reader')
        # Listen to the /scan topic.
        # Every time a scan arrives, run scan_callback.
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        # ranges[0] is the ray pointing straight ahead
        front = msg.ranges[0]
        if front == float('inf'):
            print('Front: open space (nothing in range)')
        else:
            print(f'Front distance: {front:.2f} meters')


def main():
    rclpy.init()              # start ROS 2 inside this program
    node = LidarReader()      # create our node
    try:
        rclpy.spin(node)      # run forever, waiting for messages
    except KeyboardInterrupt:
        pass                  # Ctrl+C = clean exit


if __name__ == '__main__':
    main()
```

Save and exit. Now run it (with the world, robot and bridge still running):

```bash
python3 ~/ros2_ws/src/simulation/read_lidar.py
```

Expected output:

```text
Front distance: 1.90 meters
Front distance: 1.90 meters
Front distance: 1.90 meters
...
```

Ten lines per second — the update rate of our LiDAR.

## The code, line by line

* `import rclpy` — the ROS 2 Python library. Its name means "ROS Client Library for PYthon".
* `from sensor_msgs.msg import LaserScan` — the LaserScan message type from section 10, as a Python class.
* `class LidarReader(Node):` — our program is a ROS 2 **node** (remember: node = one small program in the robot system).
* `super().__init__('lidar_reader')` — give our node its name.
* `self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)` — subscribe! In words: "messages of type LaserScan, from topic /scan, please call my function scan_callback for each one" (the 10 is a queue size — how many messages to keep if we are slow; just use 10).
* `def scan_callback(self, msg):` — this function runs every time a scan arrives. `msg` is the whole LaserScan message.
* `front = msg.ranges[0]` — pick ray number 0: straight ahead.
* `if front == float('inf')` — infinity means "nothing in range". We check it so we do not print silly text like "inf meters".
* `rclpy.spin(node)` — keep the program alive, processing messages, until you press Ctrl+C.

Drive the robot around with teleop (in another terminal) and watch the printed number change. Your program is *sensing*.

Stop it with `Ctrl+C`.

---

# 14. First intelligent behavior

Now we close the loop. The robot's first decision:

```text
          LiDAR
             │
             ▼
      Check front distance
             │
       ┌─────┴─────┐
       │           │
     Safe        Too close
       │           │
       ▼           ▼
   FORWARD        STOP
```

This is genuinely how intelligent behavior starts: **sense → decide → act**.

* Sense: read `/scan`.
* Decide: is the front distance bigger than a safe distance?
* Act: publish forward on `/cmd_vel`, or publish stop.

Our previous program could sense. The next program will also decide and act.

---

# 15. Make the robot avoid obstacles

Stopping is safe but boring. Let us upgrade the decision:

```text
              LiDAR
                │
                ▼
       Is obstacle close?
          /           \
        NO             YES
        │               │
        ▼               ▼
     FORWARD       Check left/right
                        │
                 ┌──────┴──────┐
                 │             │
              LEFT OPEN     RIGHT OPEN
                 │             │
                 ▼             ▼
              TURN LEFT     TURN RIGHT
```

In words: drive forward until something is ahead, then turn toward whichever side has more space, then drive forward again.

This is a very simple obstacle avoidance system. It is not advanced navigation — the robot has no map, no goal, no memory. It only reacts to what it sees right now. But it is enough to wander around a room without hitting anything, and it is the foundation everything else builds on.

---

# 16. Use /cmd_vel

Here is the complete connection — **the most important diagram in this lesson**:

```text
LiDAR
  │
  ▼
/scan
  │
  ▼
Python Node
  │
  │ decision
  ▼
/cmd_vel
  │
  ▼
Robot
```

```text
/scan
```

contains sensor information.

```text
/cmd_vel
```

contains movement commands.

**The Python node connects the two.**

In document 02, the thing publishing `/cmd_vel` was *you* (the keyboard). Now we replace you with a program. Nothing else changes — the robot cannot tell the difference. That swap is what "autonomous" means.

Here is the full program. Create it:

```bash
nano ~/ros2_ws/src/simulation/avoid_obstacles.py
```

Paste:

```python
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ===== Settings — change these in the final challenge! =====
FORWARD_SPEED = 0.3   # meters per second
TURN_SPEED = 0.5      # radians per second
SAFE_DISTANCE = 0.8   # start turning when something is closer (meters)
STOP_DISTANCE = 0.3   # emergency stop distance (meters)


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')
        self.scan = None            # the latest scan (none yet)
        self.last_scan_time = 0.0   # when the latest scan arrived
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)  # decide 10 times/second

    def scan_callback(self, msg):
        # Remember the newest scan and when it arrived
        self.scan = msg
        self.last_scan_time = time.time()

    def sector_min(self, start, end):
        """Smallest valid distance among rays start..end."""
        values = []
        for r in self.scan.ranges[start:end]:
            # Keep only sensible values (inside the sensor's limits)
            if self.scan.range_min < r < self.scan.range_max:
                values.append(r)
        if not values:
            return float('inf')   # no hits = open space
        return min(values)

    def control_loop(self):
        cmd = Twist()   # all zeros = stop

        # SAFETY: no sensor data, or data older than half a second? STOP.
        if self.scan is None or time.time() - self.last_scan_time > 0.5:
            self.cmd_pub.publish(cmd)
            return

        # Ray 0 = front. The front sector wraps around ray 0,
        # so we check rays 340..359 and 0..19 (about 40 degrees wide).
        front = min(self.sector_min(0, 20), self.sector_min(340, 360))
        left = self.sector_min(60, 100)     # around ray 90
        right = self.sector_min(260, 300)   # around ray 270

        if front < STOP_DISTANCE:
            # SAFETY: extremely close! Do not move forward at all.
            # Turn in place toward the side with more space.
            cmd.linear.x = 0.0
            cmd.angular.z = TURN_SPEED if left > right else -TURN_SPEED
        elif front < SAFE_DISTANCE:
            # Obstacle ahead: turn toward the open side
            if left > right:
                cmd.angular.z = TURN_SPEED      # positive = turn left
            else:
                cmd.angular.z = -TURN_SPEED     # negative = turn right
        else:
            # Path is clear: drive forward
            cmd.linear.x = FORWARD_SPEED

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ObstacleAvoider()
    print('Obstacle avoider running. Press Ctrl+C to stop.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.cmd_pub.publish(Twist())   # one last "stop" for the robot
    except Exception:
        pass


if __name__ == '__main__':
    main()
```

Save and exit.

## Understanding the new pieces

* `from geometry_msgs.msg import Twist` — the message type of `/cmd_vel`. A Twist carries `linear.x` (forward speed) and `angular.z` (turning speed).
* `self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)` — before, we only *subscribed*. Now we also **publish**. Our node writes movement commands.
* `self.create_timer(0.1, self.control_loop)` — run `control_loop` every 0.1 seconds. The robot makes 10 decisions per second.
* `sector_min(start, end)` — one ray can be noisy or unlucky. So we never trust a single ray: we look at a whole *sector* of rays and take the smallest distance (= the nearest danger) in it.
* The **safety check** at the top of `control_loop` — if sensor data is missing or old, publish stop. More about this in section 20.
* `TURN_SPEED if left > right else -TURN_SPEED` — the actual "intelligence": compare left space with right space, turn toward the bigger one.

> [!NOTE]
> If you ever need to stop a runaway robot by hand, publish one stop command:
>
> ```bash
> ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
> ```

---

# 17. Build the final environment

A single wall is too easy. Let us build a small arena:

```text
┌──────────────────────────────┐
│                              │
│     █████                    │
│     █                         │
│     █          █████         │
│                █             │
│       🤖                     │
│                              │
│                    EXIT →    │
└──────────────────────────────┘
```

Walls around the edge, obstacles inside, and one open exit on the right.

Create the world:

```bash
nano ~/ros2_ws/src/simulation/obstacle_world.sdf
```

Paste:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="obstacle_world">

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- The arena: one static model, many wall links -->
    <model name="arena">
      <static>true</static>

      <!-- Outer walls (8m x 8m room) -->
      <link name="wall_north">
        <pose>0 4 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry>
          <material><ambient>0.3 0.5 0.8 1</ambient>
                    <diffuse>0.3 0.5 0.8 1</diffuse></material>
        </visual>
      </link>

      <link name="wall_south">
        <pose>0 -4 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry>
          <material><ambient>0.3 0.5 0.8 1</ambient>
                    <diffuse>0.3 0.5 0.8 1</diffuse></material>
        </visual>
      </link>

      <link name="wall_west">
        <pose>-4 0 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>0.2 8.4 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>0.2 8.4 0.5</size></box></geometry>
          <material><ambient>0.3 0.5 0.8 1</ambient>
                    <diffuse>0.3 0.5 0.8 1</diffuse></material>
        </visual>
      </link>

      <!-- East wall has a 2 meter gap in the middle: the EXIT -->
      <link name="wall_east_top">
        <pose>4 2.5 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>0.2 3 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>0.2 3 0.5</size></box></geometry>
          <material><ambient>0.3 0.5 0.8 1</ambient>
                    <diffuse>0.3 0.5 0.8 1</diffuse></material>
        </visual>
      </link>

      <link name="wall_east_bottom">
        <pose>4 -2.5 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>0.2 3 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>0.2 3 0.5</size></box></geometry>
          <material><ambient>0.3 0.5 0.8 1</ambient>
                    <diffuse>0.3 0.5 0.8 1</diffuse></material>
        </visual>
      </link>

      <!-- Obstacles inside the room -->
      <link name="obstacle_wall_1">
        <pose>-1.5 1.5 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>2 0.2 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>2 0.2 0.5</size></box></geometry>
          <material><ambient>0.8 0.3 0.3 1</ambient>
                    <diffuse>0.8 0.3 0.3 1</diffuse></material>
        </visual>
      </link>

      <link name="obstacle_wall_2">
        <pose>1.5 -0.5 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>0.2 2.5 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>0.2 2.5 0.5</size></box></geometry>
          <material><ambient>0.8 0.3 0.3 1</ambient>
                    <diffuse>0.8 0.3 0.3 1</diffuse></material>
        </visual>
      </link>

      <link name="obstacle_box">
        <pose>0.5 2.5 0.25 0 0 0</pose>
        <collision name="c">
          <geometry><box><size>1 1 0.5</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>1 1 0.5</size></box></geometry>
          <material><ambient>0.8 0.6 0.2 1</ambient>
                    <diffuse>0.8 0.6 0.2 1</diffuse></material>
        </visual>
      </link>

    </model>

  </world>
</sdf>
```

Save and exit.

The robot must move through this environment without touching anything.

---

# 18. Final project

The complete brain of your robot:

```text
START
  │
  ▼
MOVE FORWARD
  │
  ▼
READ LiDAR
  │
  ▼
OBSTACLE?
 ┌┴────────────┐
NO             YES
 │               │
 ▼               ▼
MOVE          CHECK SPACE
                 │
          ┌──────┴──────┐
          ▼             ▼
       TURN LEFT     TURN RIGHT
          │             │
          └──────┬──────┘
                 ▼
            MOVE AGAIN
```

Time to run it all. Close everything from before (`Ctrl+C` everywhere) and start fresh:

**Terminal 1** — the arena:

```bash
gz sim -r ~/ros2_ws/src/simulation/obstacle_world.sdf
```

**Terminal 2** — spawn the robot in the bottom-left corner:

```bash
ros2 run ros_gz_sim create -file ~/ros2_ws/src/simulation/my_robot_lidar.urdf -name my_robot -x -2.5 -y -2.5 -z 0.2
```

**Terminal 3** — the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist' \
  '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry' \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V' \
  '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

**Terminal 4** — robot shape (for RViz):

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat ~/ros2_ws/src/simulation/my_robot_lidar.urdf)" \
  -p use_sim_time:=true
```

**Terminal 5** — RViz (Fixed Frame `odom`, add RobotModel + TF + LaserScan like before):

```bash
rviz2 --ros-args -p use_sim_time:=true
```

**Terminal 6** — the brain:

```bash
python3 ~/ros2_ws/src/simulation/avoid_obstacles.py
```

### What should happen?

The robot drives forward on its own. When it comes close to a wall or an obstacle, it turns toward open space and keeps going. It wanders around the arena — and sooner or later, it finds the exit gap and escapes.

Watch RViz at the same time: the ring of laser dots shows exactly what the robot sees at every moment, and you can literally watch it "notice" each wall before turning.

Nobody is touching the keyboard. **The robot is autonomous.**

---

# 19. Final challenge

Open `avoid_obstacles.py` and look at the settings at the top:

```text
FORWARD_SPEED
TURN_SPEED
SAFE_DISTANCE
```

Change them, one at a time. Restart the node (Ctrl+C, then run again) and observe what happens.

**Can you make the robot turn earlier?**

(Hint: a bigger `SAFE_DISTANCE` means the robot reacts while the obstacle is still far away.)

**Can you make it move faster?**

(Hint: raise `FORWARD_SPEED` — but careful: a fast robot with a small `SAFE_DISTANCE` cannot turn in time. Try 1.0 m/s and watch it struggle.)

**Can you make it work in a smaller room?**

(Edit `obstacle_world.sdf`: change the outer walls from 8 meters to 4 meters — the poses `±4` become `±2` and the wall lengths shrink. A smaller room needs a slower robot with earlier turns. Find settings that work!)

There is no single right answer. Tuning these numbers *is* robotics.

---

# 20. Important safety idea

Look again at this part of `avoid_obstacles.py`:

```python
# SAFETY: no sensor data, or data older than half a second? STOP.
if self.scan is None or time.time() - self.last_scan_time > 0.5:
    self.cmd_pub.publish(cmd)
    return
```

This is a **safety stop**, and it is one of the most important ideas in robotics:

```text
If sensor data is missing
        ↓
STOP
```

```text
If obstacle is extremely close
        ↓
STOP
```

(That second rule is our `STOP_DISTANCE` check.)

Why does this matter? A blind robot must never move. If the LiDAR breaks, or the cable disconnects, or the program crashes — the *worst* possible response is "keep doing what I was doing".

Our simulated robot cannot hurt anyone. But a real robot is heavy, fast, and shares space with people. Real robots need safety systems too — always, and usually several layers of them. You just built your first one.

Try it: while the robot is wandering, press `Ctrl+C` in the **bridge** terminal (Terminal 3). The `/scan` data stops flowing... and within half a second, the robot stops. Restart the bridge and it continues. That is your safety stop working.

---

# 21. What you have built

```text
             YOUR ROBOT

              LiDAR
                │
                ▼
              /scan
                │
                ▼
             ROS 2
                │
         ┌──────┴──────┐
         │             │
      Decision       RViz
         │
         ▼
      /cmd_vel
         │
         ▼
       Gazebo
         │
         ▼
       Robot
```

You started with an empty simulation.

You created a world.

You created a robot.

You made the robot move.

You added a LiDAR.

You connected the LiDAR to ROS 2.

You viewed the sensor in RViz.

You wrote a small program that reads the sensor.

You made the robot make a decision.

Now your simulated robot can move around obstacles.

---

Every autonomous robot on Earth — warehouse robots, delivery robots, self-driving cars — is built on the loop you just made: **sense → decide → act**. They have better sensors, better maps, and much smarter decisions. But the loop is the same.

In a future module we will give this robot a map and a goal (this is called SLAM and navigation). For now, be proud: your robot sees, thinks, and moves — and you understand every piece of how it does it.
