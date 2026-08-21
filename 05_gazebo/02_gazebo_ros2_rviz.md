# Gazebo + ROS 2 + RViz — Understanding the Simulation

In the last document you installed everything.

In this document you will:

* build your first world
* build your first robot
* drive the robot with your keyboard
* watch the robot in RViz

Take your time. Every section builds on the previous one.

> [!NOTE]
> Reminder: run `source /opt/ros/jazzy/setup.bash` in **every** new terminal
> (unless it is already in your `~/.bashrc`).

---

# 1. What are we building?

Here is the complete picture of today's system:

```text
                    ROS 2
                      │
          ┌───────────┼───────────┐
          │           │           │
       /cmd_vel      /odom       /scan
          │           ↑           ↑
          │           │           │
          ▼           │           │
       Gazebo ────────┴───────────┘
          │
          ▼
        Robot
          │
       Sensors
```

In simple words:

* **Gazebo** simulates the world and the robot.
* **ROS 2** carries messages between programs.
* **/cmd_vel** — movement commands go *into* the robot ("drive forward!").
* **/odom** — position information comes *out* of the robot ("I moved 2 meters").
* **/scan** — sensor data comes *out* of the robot (we add this in document 03).

Today we build everything except `/scan`. That one gets its own document.

---

# 2. Gazebo

Gazebo creates the virtual world.

Everything physical lives inside Gazebo:

```text
Ground
Walls
Robot
Obstacles
Sensors
```

Gazebo also simulates physics: gravity pulls things down, wheels grip the ground, objects crash into each other.

Think of Gazebo as **the world itself**.

---

# 3. ROS 2

ROS 2 allows different parts of our robot system to communicate.

A robot system is many small programs. ROS 2 calls each program a **node**.

Nodes talk to each other through **topics**:

```text
Node A
  │
  │ "Move forward"
  ▼
Topic
  │
  ▼
Node B
```

A topic is like a named channel. One node writes into it, another node reads from it.

For our robot:

```text
Keyboard node
     │
     │ writes "go forward" into topic /cmd_vel
     ▼
  /cmd_vel
     │
     │ Gazebo reads from /cmd_vel
     ▼
  Gazebo moves the robot's wheels
```

Think of ROS 2 as **the nervous system**.

---

# 4. RViz

RViz helps us see what the robot understands.

This is the most important comparison in this whole course:

```text
GAZEBO
What is happening in the virtual world?

RVIZ
What does the robot understand?
```

Gazebo is the *truth*. RViz is the *robot's brain view*.

Example: if a wall exists in Gazebo but the robot has no sensor to see it, the wall will **not** appear in RViz. The robot does not know about it!

RViz is not a physics simulator. It simulates nothing. It only **displays** data that arrives through ROS 2 topics.

Think of RViz as **the robot's imagination on a screen**.

---

# 5. Create the first world

## What is a world?

A world is a file that describes everything in the simulation:

```text
Ground
+
Light
+
Box
+
Wall
```

World files are written in a format called **SDF** (Simulation Description Format). It is a text file. You can read it and edit it.

Everything you place in a world is called a **model**. The ground is a model. A box is a model. Later, our robot is a model too.

## Position: X, Y, Z

Every model has a position, written as three numbers:

```text
        Z (up)
        │
        │
        └──────── X (forward)
       /
      /
     Y (left)

X = forward / backward
Y = left / right
Z = up / down
```

Example: `<pose>3 0 0.5 0 0 0</pose>` means: 3 meters forward, 0 sideways, 0.5 meters up.

## Rotation: Roll, Pitch, Yaw

A model can also be rotated. The last three numbers of a pose are the rotation:

```text
Roll   = tilt sideways      (like a plane rolling)
Pitch  = tilt forward/back  (like nodding your head)
Yaw    = turn left/right    (like shaking your head)
```

```text
Pose = X  Y  Z  Roll  Pitch  Yaw
       │  │  │   │      │     │
       position  └── rotation ┘
```

For a ground robot, **yaw** is the important one. Yaw is measured in radians (3.14 radians = 180 degrees).

## Try it — create the world file

Create the file with the `nano` text editor (or use any editor you like, for example VS Code):

```bash
nano ~/ros2_ws/src/simulation/first_world.sdf
```

Paste all of this into the file:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="first_world">

    <!-- LIGHT: like the sun. Without light the world is dark. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- GROUND: a big flat floor -->
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

    <!-- BOX: 1x1x1 meter cube, 3 meters in front of the start point -->
    <model name="box_1">
      <static>true</static>
      <pose>3 0 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="box_collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.3 0.3 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- WALL: 6 meters long, 4 meters to the left -->
    <model name="wall_1">
      <static>true</static>
      <pose>0 4 0.5 0 0 0</pose>
      <link name="wall_link">
        <collision name="wall_collision">
          <geometry>
            <box><size>6 0.2 1</size></box>
          </geometry>
        </collision>
        <visual name="wall_visual">
          <geometry>
            <box><size>6 0.2 1</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.5 0.8 1</ambient>
            <diffuse>0.3 0.5 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

Save and exit nano: press `Ctrl+O`, then `Enter`, then `Ctrl+X`.

Some words from the file, explained:

```text
<world>      →  the whole world
<model>      →  one object in the world
<static>     →  true = this object never moves (walls, ground)
<link>       →  one solid part of a model
<collision>  →  the invisible shape used for physics (what you crash into)
<visual>     →  the shape you SEE (usually the same as collision)
<pose>       →  position + rotation: X Y Z Roll Pitch Yaw
```

Why both `<collision>` and `<visual>`? Because looks and physics are separate. A video game character can have beautiful hair (visual) but the game only checks crashes against a simple capsule (collision). Same idea here.

Now run your world:

```bash
gz sim -r ~/ros2_ws/src/simulation/first_world.sdf
```

The `-r` means "run immediately" — do not start paused.

### What should happen?

You see a grey floor, a red box in front, and a blue wall on the left side.

### Challenge

Change the box position from `3 0 0.5` to `3 2 0.5` and restart Gazebo.

Where did the box move? (2 meters to the left — Y is the left direction.)

---

# 6. Understanding physics

Gazebo does not just draw objects. It simulates physics.

**Gravity** — everything is pulled down.

```text
   ▢   ← box in the air
   │
   ▼   gravity
──────── ground
```

**Collision** — two objects cannot pass through each other. A robot driving into a wall stops (or crashes).

**Friction** — surfaces grip each other. Without friction, wheels would spin in place like on ice. Our robot *needs* friction to drive.

**Mass** — how heavy an object is. Heavy objects are harder to push. Every moving object in Gazebo must have a mass.

### Try it

Open your world again, pause the simulation (⏸), and use the toolbar to add a box **above the ground** (click the box icon, then click in the air... it will drop onto the ground when you press play ▶).

That falling? That is gravity + collision working together.

---

# 7. Build a simple robot

Time for the robot. Do not worry — we start with the simplest robot possible:

```text
Robot
│
├── Body
├── Left wheel
└── Right wheel
```

## Links and joints

A robot description uses two building blocks:

```text
LINK = robot part

JOINT = connection between parts
```

```text
   base_link (the body)
   │            │
 joint        joint
   │            │
left_wheel   right_wheel
```

A joint also says *how* the parts can move. Our wheel joints can rotate forever (like real wheels). The description calls this a **continuous** joint.

## What is URDF?

**URDF** means Unified Robot Description Format.

It is a text file that describes a robot: its parts, its joints, its sizes, its weights.

```text
URDF file
    │
    │  "body is a 40cm box, wheels are 10cm cylinders..."
    ▼
Gazebo builds the robot in the world
    │
    ▼
RViz can draw the robot on screen
```

One file, used by both Gazebo and RViz. That is why we use URDF.

(You may also see robots written directly in SDF. That works too. URDF is the common ROS 2 way, so we use it.)

## Try it — create the robot file

```bash
nano ~/ros2_ws/src/simulation/my_robot.urdf
```

Paste all of this:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- ================== BODY ================== -->
  <!-- A box: 40cm long, 30cm wide, 10cm tall -->
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
  <!-- A cylinder: 10cm radius, 5cm thick -->
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
  <!-- A small sliding ball at the front so the robot does not tip over -->
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

  <!-- Make the caster slippery, so it slides instead of gripping -->
  <gazebo reference="caster">
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>

  <!-- ================== GAZEBO PLUGINS ================== -->
  <!-- These give the robot its "motor controller" and joint reporting -->
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

That is a lot of text! Let us break it down:

```text
<link>       →  one robot part (body, wheel, caster)
<visual>     →  what the part looks like
<collision>  →  the shape used for physics
<inertial>   →  the mass and how it is spread out
<joint>      →  connects two links
  continuous →  can rotate forever (wheels)
  fixed      →  glued, never moves (caster holder)
<axis>       →  which direction the joint rotates around
               (0 1 0 = the Y axis = sideways = how a wheel spins)
```

Some details worth understanding:

* `rpy="1.5708 0 0"` on the wheels: cylinders in URDF stand upright by default. 1.5708 radians = 90 degrees. We lay the cylinder on its side so it looks like a wheel.
* The **caster** is a slippery ball at the front. Two wheels alone would let the robot tip forward. The caster holds the front up but does not grip the ground.
* The `<gazebo>` blocks at the end are instructions only Gazebo reads:
  * **DiffDrive plugin** — the robot's motor controller. It listens on the topic `cmd_vel` and spins the wheel joints. It also reports the position on `odom` (more about this soon).
  * **JointStatePublisher plugin** — reports how far each wheel has turned. RViz needs this to draw the spinning wheels.

You do NOT need to memorize this file. You need to *recognize* its parts.

---

# 8. Spawn the robot in Gazebo

**Spawn** means: place the robot into the running world.

**Terminal 1** — start your world:

```bash
gz sim -r ~/ros2_ws/src/simulation/first_world.sdf
```

**Terminal 2** — spawn the robot:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_sim create -file ~/ros2_ws/src/simulation/my_robot.urdf -name my_robot -x 0 -y 0 -z 0.2
```

What does this command do?

```text
ros2 run ros_gz_sim create
   ↓
run the "create" tool from the ros_gz_sim package.
It sends your robot file into the running Gazebo world.

-file ...my_robot.urdf
   ↓
which robot file to use

-name my_robot
   ↓
the robot's name inside the world

-x 0 -y 0 -z 0.2
   ↓
where to place it (20cm above the ground —
it falls that tiny distance and lands on its wheels)
```

### What should happen?

```text
Gazebo

        🤖
────────────────────
       Ground
```

A small blue box with two black wheels appears in the middle of the world. It sits still. It has a motor controller, but nobody is sending commands yet.

---

# 9. Make the robot move

Our robot uses **differential drive**. Big words, simple idea:

Two wheels, each with its own motor. That is all.

```text
LEFT WHEEL        RIGHT WHEEL

    ↻                 ↻

       FORWARD
```

The magic is in the speeds:

```text
Left = same speed
Right = same speed

       ↓

Robot moves forward
```

```text
Left = fast
Right = slow

       ↓

Robot turns right
```

```text
Left = backward
Right = forward

       ↓

Robot spins in place (turns left)
```

No steering wheel. No turning mechanism. Just two wheel speeds. Most small robots (including robot vacuum cleaners) work exactly like this.

The DiffDrive plugin in our URDF does the math: you say "forward at 0.5 m/s and turn a little", it calculates the two wheel speeds.

---

# 10. ROS 2 /cmd_vel

How do we tell the robot to move? Through a topic called `/cmd_vel`.

`/cmd_vel` means "command velocity" — the speed we *want*.

```text
Keyboard
   │
   ▼
/cmd_vel
   │
   ▼
Robot
```

Each `/cmd_vel` message carries two important numbers:

```text
linear.x   →  forward speed (meters per second)
angular.z  →  turning speed (radians per second)
```

## Start the bridge

Remember from document 01: Gazebo and ROS 2 are different systems. They need the translator (`ros_gz_bridge`).

**Terminal 3** — start the bridge (keep Terminals 1 and 2's world and robot running):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist' \
  '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry' \
  '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V' \
  '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

This looks scary. It is not. Each line is one topic being translated:

```text
/cmd_vel       →  movement commands     (ROS 2 → Gazebo)
/odom          →  robot position        (Gazebo → ROS 2)
/tf            →  robot part positions  (Gazebo → ROS 2)
/joint_states  →  wheel rotation        (Gazebo → ROS 2)
/clock         →  simulation time       (Gazebo → ROS 2)
```

The pattern of each line is:

```text
topic_name @ ROS-message-type @ Gazebo-message-type
```

The `@` means "translate in both directions". The `[` on the clock line means "one direction only: Gazebo → ROS 2".

Leave this terminal running. The bridge must stay alive the whole time.

## Drive with the keyboard

Install the keyboard driving tool (one time only):

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard -y
```

**Terminal 4** — drive!

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The keys:

```text
   u    i    o
   j    k    l
   m    ,    .

i = forward        , = backward
j = turn left      l = turn right
k = STOP
q / z = faster / slower
```

> [!NOTE]
> Click on Terminal 4 first! The keyboard only works while that terminal window is selected.

Press `i`. Watch Gazebo. **Your robot drives!**

Drive it into the red box. It crashes and stops. That is collision physics.

## Inspect the topic

**Terminal 5** — see what is flowing:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

You should see (among others):

```text
/clock
/cmd_vel
/joint_states
/odom
/tf
```

Now watch the movement commands live:

```bash
ros2 topic echo /cmd_vel
```

Press `i` in the teleop terminal. In Terminal 5 you see:

```text
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

`linear.x: 0.5` = drive forward at 0.5 meters per second. That message traveled: keyboard → `/cmd_vel` → bridge → Gazebo → wheels.

Stop the echo with `Ctrl+C`.

---

# 11. ROS 2 topics

You have now used topics several times. Let us name the idea properly:

```text
Topic
   ↓
A communication channel
```

A topic has a name and a message type. Any node can publish (write). Any node can subscribe (read).

The topics of our robot:

```text
/cmd_vel        →  "please move like this"       (commands IN)
/odom           →  "here is how I have moved"    (position OUT)
/scan           →  "here is what my LiDAR sees"  (sensors OUT — document 03)
/tf             →  "here is where my parts are"  (positions OUT)
```

That is all you need to know about topics for now. You will use them constantly, and they will become natural.

---

# 12. Odometry

Odometry is the robot's estimate of how it has moved.

```text
START
  🤖
   │
   │ move
   ▼
   🤖
END
```

How does the robot know it moved? It counts wheel rotations.

"My wheels are 10cm radius. They turned 3 full turns. So I moved about 1.88 meters."

This estimate is published on the `/odom` topic. Our DiffDrive plugin computes it automatically.

### Try it

While driving the robot around (teleop in Terminal 4), run in Terminal 5:

```bash
ros2 topic echo /odom
```

You will see a lot of numbers. Look for this part:

```text
pose:
  pose:
    position:
      x: 1.2
      y: 0.3
```

The numbers describe the robot's movement: "I am 1.2 meters forward and 0.3 meters left of where I started."

Drive forward and watch `x` grow. That is odometry.

Stop the echo with `Ctrl+C`.

One honest warning: odometry is an *estimate*. If the wheels slip, the estimate becomes wrong. Real robots combine odometry with other sensors. For us, the estimate is good enough.

---

# 13. TF

TF means "transform". Simple meaning:

**TF tells ROS 2 where one robot part is compared to another part.**

```text
odom
  │
  └── base_link
        │
        ├── left_wheel
        ├── right_wheel
        └── caster
```

Read it like a family tree:

* "Where is the robot (`base_link`) compared to its start point (`odom`)?" → that comes from odometry.
* "Where is the left wheel compared to the body?" → that comes from the URDF file.

Why does this matter? Imagine the LiDAR (next document) says "wall at 2 meters". Two meters from *what*? From the LiDAR. But where is the LiDAR? TF answers that chain of questions, so ROS 2 can say "the wall is at this exact spot in the world".

We do not need TF mathematics. We only need the TF tree to *exist*, and it is built automatically from our URDF and our plugins.

---

# 14. RViz setup

Now the fun part: seeing the world through the robot's eyes.

We need one more helper first. RViz wants to know the robot's shape. A standard ROS 2 node called `robot_state_publisher` reads our URDF and shares it.

**Terminal 6:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat ~/ros2_ws/src/simulation/my_robot.urdf)" \
  -p use_sim_time:=true
```

What does this do?

```text
robot_state_publisher
   ↓
reads the URDF
   ↓
tells everyone the robot's shape (topic /robot_description)
   ↓
publishes TF for all the robot's parts
```

```text
"$(cat ...my_robot.urdf)"
   ↓
put the whole content of the URDF file into the parameter

use_sim_time:=true
   ↓
"use the simulation clock, not the wall clock"
(the /clock topic from our bridge — this keeps
 Gazebo and ROS 2 time in sync)
```

**Terminal 7** — start RViz (also on simulation time):

```bash
source /opt/ros/jazzy/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

## Set up RViz step by step

**1. Fixed Frame.** Top-left panel, under "Global Options", find **Fixed Frame**. Click the field and change `map` to `odom`.

```text
Fixed Frame
   ↓
"which point of view should RViz use?"
odom = the robot's starting point
```

**2. Add the robot model.**

```text
RViz
 ↓
Add   (button, bottom-left)
 ↓
RobotModel
 ↓
OK
```

Then in the left panel, open **RobotModel** → find **Description Topic** → set it to `/robot_description`.

Your robot appears! Blue body, black wheels — drawn from the same URDF file.

**3. Add TF.**

```text
Add
 ↓
TF
 ↓
OK
```

You now see small colored axes on every robot part. Red = X, green = Y, blue = Z. That is the TF tree made visible.

**4. Grid** — already there by default. It just helps you see distances (each cell = 1 meter).

(We skip **LaserScan** today. We have no laser yet. Next document!)

---

# 15. Connect Gazebo data to RViz

Wait — how did RViz know where the robot is?

```text
Gazebo
  │
  │ sensor data
  ▼
ROS 2
  │
  ▼
RViz
```

Follow the chain:

1. Gazebo simulates the robot and computes its position.
2. The bridge (Terminal 3) translates `/tf`, `/odom`, `/joint_states` into ROS 2.
3. `robot_state_publisher` adds the robot's shape.
4. RViz subscribes to all of it and draws the picture.

RViz never talks to Gazebo directly. Everything flows through ROS 2 topics.

This is important: RViz shows only what arrives on topics. Notice the red box and the blue wall are NOT in RViz. The robot has no sensor to detect them — so as far as the robot knows, **they do not exist**. In the next document we give the robot eyes.

---

# 16. Activity

Do the complete run, from zero. Close everything (`Ctrl+C` in all terminals) and start fresh:

1. **Start Gazebo** (Terminal 1):
   ```bash
   gz sim -r ~/ros2_ws/src/simulation/first_world.sdf
   ```
2. **Start the robot** (Terminal 2):
   ```bash
   ros2 run ros_gz_sim create -file ~/ros2_ws/src/simulation/my_robot.urdf -name my_robot -x 0 -y 0 -z 0.2
   ```
3. **Start the bridge** (Terminal 3):
   ```bash
   ros2 run ros_gz_bridge parameter_bridge \
     '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist' \
     '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry' \
     '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V' \
     '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model' \
     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
   ```
4. **Start robot_state_publisher** (Terminal 4):
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args \
     -p robot_description:="$(cat ~/ros2_ws/src/simulation/my_robot.urdf)" \
     -p use_sim_time:=true
   ```
5. **Start RViz** (Terminal 5): `rviz2 --ros-args -p use_sim_time:=true`
6. **Add RobotModel** (Description Topic = `/robot_description`) **and TF**, Fixed Frame = `odom`
7. **Move the robot** (Terminal 6): `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
8. **Observe the robot in RViz** while you drive it in Gazebo

Yes, that is six terminals. Real robot work looks exactly like this. (Later courses teach "launch files" which start everything with one command — but first you should know what the pieces are.)

### What should happen?

Put the Gazebo window and the RViz window side by side.

Drive with the keyboard. The robot moves in **both** windows, at the same time, with spinning wheels.

Gazebo shows the world. RViz shows what the robot knows.

---

# 17. Activity challenge

**Challenge 1:** Can you make the robot move forward and watch its position change?

(Drive with `i`, and watch `ros2 topic echo /odom` in a spare terminal — the `position: x:` number should grow.)

**Challenge 2:** Can you turn the robot?

(Press `j` or `l`. In RViz, watch the TF axes rotate.)

**Challenge 3:** Can you identify `/cmd_vel` and `/odom`?

Run:

```bash
ros2 topic list
```

Point at each topic and say out loud what it carries. If you can explain `/cmd_vel` and `/odom` to a friend, you have understood this document.

---

# 18. Summary

```text
Gazebo
  ↓
Simulates the world

ROS 2
  ↓
Connects the robot systems

RViz
  ↓
Shows what the robot understands
```

You built a world. You built a robot. You drove it with your keyboard. You watched it through the robot's own eyes in RViz.

But your robot is blind. It only knows how far its wheels have turned. It cannot see the box. It cannot see the wall.

```text
Next:

We will give our robot
a LiDAR sensor.
```

**Next document:** [Build a Robot with LiDAR — ROS 2 Simulation](03_robot_lidar_navigation.md)
