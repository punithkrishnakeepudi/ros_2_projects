# Gazebo Installation — Beginner Setup

Welcome!

In this document we will install Gazebo and connect it to ROS 2.

You do not need to know anything about Gazebo yet.

We will go slowly, one step at a time.

---

## 1. What is Gazebo?

Gazebo lets us create a robot inside the computer.

The robot is not real.

Gazebo creates a virtual world where we can test the robot.

This is called **simulation**.

Simulation means: the computer pretends to be the real world.

A real robot works like this:

```text
REAL ROBOT

Sensor
  ↓
Computer
  ↓
Motor
  ↓
Robot
```

A simulated robot works like this:

```text
SIMULATION

Virtual Sensor
      ↓
    Gazebo
      ↓
Virtual Robot
```

### Why is simulation useful?

* A real robot is expensive. A simulated robot is free.
* If a simulated robot crashes into a wall, nothing breaks.
* We can test our code 100 times without touching hardware.
* We can build worlds that do not exist: mazes, warehouses, other planets.

First we learn in simulation.

Later, the same code can run on a real robot.

### One important note about the name "Gazebo"

Older tutorials on the internet may say:

```text
Gazebo Classic
```

Newer Gazebo uses:

```text
Gazebo / Gazebo Sim
```

They are different programs.

For this course we use:

```text
Gazebo Harmonic
```

Gazebo Harmonic is the modern Gazebo, and it is the version made for ROS 2 Jazzy.

If an internet tutorial uses the command `gazebo`, it is the old version. Our command will be `gz sim`.

---

## 2. What will we install?

Here is our full setup:

```text
Ubuntu 24.04
      ↓
ROS 2 Jazzy
      ↓
Gazebo Harmonic
      ↓
ROS 2 ↔ Gazebo integration
      ↓
RViz2
```

In simple words:

* **Ubuntu 24.04** — the operating system. You should already have this (see the [setup guide](../setup/rosJazzy_setup.md)).
* **ROS 2 Jazzy** — the robot software framework. You should already have this too.
* **Gazebo Harmonic** — the simulator. It creates the virtual world. We install this today.
* **ROS 2 ↔ Gazebo integration** — a "translator" so ROS 2 and Gazebo can talk to each other.
* **RViz2** — a viewer that shows what the robot *thinks*. It comes with ROS 2 Jazzy.

> [!NOTE]
> **Very important habit:** every time you open a new terminal, run this first:
>
> ```bash
> source /opt/ros/jazzy/setup.bash
> ```
>
> This "turns on" ROS 2 in that terminal.
> If you added this line to your `~/.bashrc` during the ROS 2 setup, it happens automatically.

---

## 3. Check Ubuntu version

Run:

```bash
lsb_release -a
```

What does this command do?

```text
lsb_release
   ↓
prints information about your Ubuntu version
```

Example output:

```text
Distributor ID: Ubuntu
Description:    Ubuntu 24.04.1 LTS
Release:        24.04
Codename:       noble
```

**What should you look for?**

The line `Release:` must say `24.04`.

If it says something else (for example `22.04`), stop here.

Gazebo Harmonic + ROS 2 Jazzy need Ubuntu 24.04.

Follow the [ROS Jazzy setup guide](../setup/rosJazzy_setup.md) first.

---

## 4. Check ROS 2

Run:

```bash
printenv ROS_DISTRO
```

What does this command do?

```text
printenv
   ↓
prints an environment variable

ROS_DISTRO
   ↓
tells us which ROS 2 version is active
```

Expected output:

```text
jazzy
```

If you see nothing, ROS 2 is not active in this terminal.

Fix it:

```bash
source /opt/ros/jazzy/setup.bash
printenv ROS_DISTRO
```

Now do one more check. Ask ROS 2 to list its topics:

```bash
ros2 topic list
```

Expected output:

```text
/parameter_events
/rosout
```

These two topics always exist.

If you see them, ROS 2 works.

---

## 5. Install Gazebo Harmonic

Good news: for Ubuntu 24.04 + ROS 2 Jazzy, there is one command that installs everything.

**Step 1 — Update the package list:**

```bash
sudo apt update
```

What does it do?

```text
sudo
   ↓
run as administrator

apt update
   ↓
refresh the list of available software
```

**Step 2 — Install Gazebo Harmonic together with the ROS 2 integration:**

```bash
sudo apt install ros-jazzy-ros-gz -y
```

What does it do?

```text
apt install
   ↓
download and install software

ros-jazzy-ros-gz
   ↓
the official "ROS 2 Jazzy + Gazebo" package.
It installs Gazebo Harmonic AND the translator
packages that connect Gazebo to ROS 2.

-y
   ↓
answer "yes" automatically
```

This may take a few minutes. It downloads a lot of packages. That is normal.

**Why only one command?**

ROS 2 Jazzy and Gazebo Harmonic were released as a matching pair.

The ROS 2 package servers already contain the correct Gazebo version.

So we do not need to add extra download sources.

---

## 6. Check Gazebo

First check the version:

```bash
gz sim --version
```

> [!NOTE]
> If the terminal says `gz: command not found`, run `source /opt/ros/jazzy/setup.bash` and try again.

Expected output (the last numbers may differ):

```text
Gazebo Sim, version 8.9.0
```

Version 8 = Gazebo Harmonic. 

Now start Gazebo with a demo world:

```bash
gz sim shapes.sdf
```

What does it do?

```text
gz sim
   ↓
start the Gazebo simulator

shapes.sdf
   ↓
a demo world file with some simple shapes
```

The first start can be slow (30–60 seconds). Be patient.

**What should you see?**

A window opens. Inside the window:

```text
┌─────────────────────────────────────┐
│  Toolbar (shapes, move, rotate)     │
├─────────────────────────────┬───────┤
│                             │ Panel │
│        3D world             │       │
│                             │ Entity│
│   ▢  ●  ▬  (some shapes)    │ tree  │
│  ───────────────────────    │       │
│        ground               │       │
├─────────────────────────────┴───────┤
│  ▶ ⏸   time, real time factor      │
└─────────────────────────────────────┘
```

The main parts:

* **3D view** — the big area in the middle. This is the virtual world.
* **Play ▶ / Pause ⏸** — bottom-left corner. Gazebo starts **paused**. Nothing moves until you press play.
* **Entity tree** — right panel. A list of everything in the world.
* **Toolbar** — top. Buttons to add shapes and move objects.

Press the **play ▶** button. The simulation time at the bottom starts counting.

Close Gazebo for now (or press `Ctrl+C` in the terminal).

---

## 7. Check ROS 2 + Gazebo integration

Here is an important idea:

**Gazebo and ROS 2 are two different programs.**

Gazebo does not speak "ROS 2 language" by itself.

We need a translator between them:

```text
ROS 2
  │
  │ communication
  ▼
ROS-Gazebo integration  (the "ros_gz" packages)
  │
  ▼
Gazebo
```

We already installed the translator in Step 5 (`ros-jazzy-ros-gz`).

Let us check that it is really there:

```bash
ros2 pkg list | grep ros_gz
```

What does it do?

```text
ros2 pkg list
   ↓
list all installed ROS 2 packages

| grep ros_gz
   ↓
only show the lines that contain "ros_gz"
```

Expected output (you may see a few more lines):

```text
ros_gz_bridge
ros_gz_image
ros_gz_interfaces
ros_gz_sim
```

The two packages we will use the most:

* **ros_gz_bridge** — the translator. It copies messages between ROS 2 and Gazebo.
* **ros_gz_sim** — helper tools, for example a tool to place a robot into a running world.

### Try the translator

Let us do a real test.

**Terminal 1** — start Gazebo and press play (or use `-r`, which means "run immediately"):

```bash
gz sim -r shapes.sdf
```

**Terminal 2** — start a small bridge that translates the Gazebo clock into a ROS 2 topic:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

Do not worry about the strange text. We will explain bridges properly in the next document.

**Terminal 3** — look at the clock from the ROS 2 side:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /clock --once
```

Expected output (numbers will differ):

```text
clock:
  sec: 42
  nanosec: 500000000
---
```

What just happened?

```text
Gazebo clock
     │
     ▼
ros_gz_bridge  (translator)
     │
     ▼
ROS 2 topic /clock
     │
     ▼
your terminal
```

ROS 2 just received data from Gazebo. The integration works!

Press `Ctrl+C` in all three terminals to stop everything.

---

## 8. Check RViz2

Run:

```bash
rviz2
```

**What is RViz?**

RViz is a viewer.

It does not simulate anything.

It shows what the robot *knows*: sensor data, robot position, robot shape.

We will use it a lot in the next documents.

**What should you see?**

A window with a dark grid opens:

```text
┌──────────────────────────────┐
│ Displays │                   │
│          │      (grid)       │
│  Grid    │    · · · · ·      │
│  ...     │    · · · · ·      │
│          │    · · · · ·      │
└──────────────────────────────┘
```

That is all we need for now. If the window opens, RViz works.

Close it with `Ctrl+C` in the terminal.

---

## 9. Create workspace

A **workspace** is a folder where we keep our robot projects.

Almost every ROS 2 tutorial uses the same folder name: `ros2_ws` (ROS 2 workspace).

Create it:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

What do these commands do?

```text
mkdir
→ creates a folder

-p
→ creates missing parent folders
  (creates ros2_ws AND ros2_ws/src in one go)

~
→ your home folder

cd
→ moves into the folder
```

Check that it worked:

```bash
ls ~/ros2_ws
```

Expected output:

```text
src
```

The `src` folder means "source". Our project files will live inside it.

---

## 10. Create simulation folder

Inside the workspace, create a folder for our simulation files:

```bash
mkdir -p ~/ros2_ws/src/simulation
```

Now the structure looks like this:

```text
ros2_ws/
└── src/
    └── simulation/
```

**Why a separate simulation folder?**

* All our world files and robot files stay in one place.
* Later projects (other courses in this repository) get their own folders.
* Nothing gets mixed up.

In this course we keep it simple: plain files in one folder.

No complicated package structures yet.

---

## 11. First Gazebo test

Time for your first real test.

Start Gazebo with an empty world:

```bash
gz sim empty.sdf
```

What should happen:

```text
Gazebo opens
      ↓
World loads
      ↓
You can see the simulation
```

You should see a flat grey ground and a sky.

Press the **play ▶** button in the bottom-left corner.

The time counter at the bottom starts running.

Congratulations — you are running a physics simulation of an (empty) world.

---

## 12. Troubleshooting

### Problem: Gazebo does not open

```text
Problem
   ↓
Window never appears, or it crashes
   ↓
Check
   ↓
Are you on WSL2 or a machine with weak graphics?
   ↓
Fix
   ↓
Force software rendering:
```

```bash
LIBGL_ALWAYS_SOFTWARE=1 gz sim shapes.sdf
```

This tells the computer to draw graphics with the CPU instead of the graphics card. It is slower but very reliable.

On WSL2, also make sure WSL is up to date. In **Windows PowerShell**:

```powershell
wsl --update
```

### Problem: `ros2: command not found`

```text
Problem
   ↓
The terminal does not know ros2
   ↓
Check
   ↓
Did you source ROS 2 in THIS terminal?
   ↓
Fix
```

```bash
source /opt/ros/jazzy/setup.bash
```

To make it automatic in every new terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Problem: `ros2` works but `gz: command not found`

```text
Problem
   ↓
ROS 2 works, Gazebo command does not
   ↓
Check
   ↓
Is ros-jazzy-ros-gz really installed?
```

```bash
ros2 pkg list | grep ros_gz
```

```text
Fix
   ↓
If the list is empty, install again:
```

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz -y
```

Then open a **new** terminal and try `gz sim --version` again.

### Problem: RViz does not open

```text
Problem
   ↓
rviz2 crashes or shows a black window
   ↓
Check
   ↓
Same graphics problem as Gazebo
   ↓
Fix
```

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

### Problem: `Package 'ros_gz_bridge' not found`

```text
Problem
   ↓
ros2 run ros_gz_bridge ... fails
   ↓
Check
   ↓
printenv ROS_DISTRO   →  must say "jazzy"
   ↓
Fix
   ↓
Source ROS 2, then reinstall the integration:
```

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-ros-gz -y
```

### Problem: Wrong ROS 2 version

```text
Problem
   ↓
printenv ROS_DISTRO shows "humble" or something else
   ↓
Check
   ↓
lsb_release -a   →  must say Ubuntu 24.04
   ↓
Fix
   ↓
This course needs Ubuntu 24.04 + ROS 2 Jazzy.
Follow the setup guide in ../setup/rosJazzy_setup.md
```

---

## 13. Activity

### Activity: Explore the Gazebo interface

Open Gazebo:

```bash
gz sim shapes.sdf
```

Find these things (do not skip this — knowing the interface saves you time later):

* **3D view** — the big area with the shapes
* **World / scene area** — the ground and sky around the shapes
* **Play ▶** — bottom-left. Press it. Simulation time starts.
* **Pause ⏸** — same place. Press it. Time freezes.
* **Add a shape** — the toolbar at the top has small icons for box, sphere and cylinder
* **Entity tree** — the right panel that lists every object in the world
* **Simulation information** — the bottom bar: sim time, real time factor (RTF)

Camera controls (try each one in the 3D view):

```text
Scroll wheel          →  zoom in / out
Left-click + drag     →  move (pan) the camera
Middle-click + drag   →  rotate the camera around a point
```

### What should happen?

You can move around the world, pause it, and play it again.

The shapes stay on the ground because gravity is being simulated.

### Challenge

Add a box to the world:

1. Find the **box icon** in the top toolbar.
2. Click it.
3. Move your mouse into the 3D view — a box follows your mouse.
4. Click on the ground to drop it there.
5. Look at the entity tree on the right. Your new box appears in the list.

Extra challenge: pause the simulation, add a box **in the air**, then press play.

What happens? (It should fall. Gravity!)

---

## 14. Completion checklist

Check every box before moving on:

```text
☐ Ubuntu 24.04            (lsb_release -a)
☐ ROS 2 Jazzy             (printenv ROS_DISTRO → jazzy)
☐ Gazebo Harmonic         (gz sim --version → version 8.x)
☐ ROS 2 ↔ Gazebo packages (ros2 pkg list | grep ros_gz)
☐ RViz2                   (rviz2 opens)
☐ ROS 2 workspace         (~/ros2_ws/src exists)
☐ simulation folder       (~/ros2_ws/src/simulation exists)
☐ Gazebo opens            (gz sim empty.sdf works)
```

```text
If everything above works:

You are ready to build your first simulation.
```

**Next document:** [Gazebo + ROS 2 + RViz — Understanding the Simulation](02_gazebo_ros2_rviz.md)
