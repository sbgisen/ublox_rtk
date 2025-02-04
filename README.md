# ublox_rtk

**ublox_rtk** provides ROS packages for Real-Time-Kinematic (RTK) positioning with Ublox receivers using an NTRIP caster.

---

## Installation

### 1. Install System Dependencies
Ensure you have the necessary system packages installed:
```bash
sudo apt install rtklib
```

### 2. Configure Serial Port Access
Add your user to the `dialout` group to access serial ports:
```bash
sudo usermod -aG dialout $USER
```
*Log out and log back in for this change to take effect.*

### 3. Clone the Repository
Navigate to your Catkin workspace and clone the repository:
```bash
cd ~/catkin_ws/src
git clone git@github.com:sbgisen/ublox_rtk.git
```

### 4. Install ROS Dependencies
Ensure `rosdep` is initialized and up-to-date to handle ROS-specific dependencies:
```bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Workspace
Compile the workspace and source the setup file:
```bash
catkin build
source ~/catkin_ws/devel/setup.bash
```

---

## Packages

### 1. **ublox_ntrip_client**
A ROS node that retrieves RTCM3 data from an NTRIP caster and outputs it to a Ublox receiver via a serial port.

---
