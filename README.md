# Adaptive Mahony Attitude Controller (ROS 2 Jazzy + PX4)

This package implements an advanced, adaptive Mahony attitude estimator for fixed-wing UAVs, integrated with ROS 2 Jazzy and PX4 Autopilot SITL in Gazebo.

## System Requirements

* **OS:** Ubuntu 24.04 (Noble Numbat)
* **ROS 2:** Jazzy Jalisco
* **Simulator:** Gazebo Harmonic (8.9.0)
* **Flight Stack:** PX4 Autopilot
* **Middleware:** Micro-XRCE-DDS Agent

## Features

* **Adaptive Gain Scheduling:** Dynamically adjusts Kp and Ki based on sensor confidence levels.
* **Confidence Metrics:** Calculates real-time confidence for Accelerometer (gravity vector), Magnetometer, and Gyroscope data.
* **Sensor Filtering:** Implements Low Pass Filters (LPF) for noisy sensor data.
* **Automatic Calibration:** Performs gyro and mag bias calculation on startup.
* **Initial Alignment:** Uses Gazebo ground truth for initial quaternion alignment and magnetic declination calculation.

## ⚠️ Critical Simulation Configuration

**You must modify the Gazebo model file for this controller to work.** The default PX4 `rc_cessna` model does not publish the ground truth odometry required for alignment, nor does it have the realistic noise or wind settings this controller is designed to handle.

**1. Locate the Model File:**
Navigate to your PX4-Autopilot directory and find the `model.sdf` for the Cessna:
`~/PX4-Autopilot/Tools/simulation/gz/models/rc_cessna/model.sdf`

**2. Add the Odometry Plugin:**

<plugin
    filename="gz-sim-odometry-publisher-system"
    name="gz::sim::systems::Odometry Publisher">
    <odom_frame>rc_cessna_0/odom</odom_frame>
    <robot_base_frame>rc_cessna_0/base_link</robot_base_frame>
    <odom_publish_frequency>50</odom_publish_frequency>
</plugin>

**Add Realistic Sensor Noise:**
 Replace the existing <sensor name="imu_sensor"> block (or add to it) to include Gaussian noise and bias. This tests the filter's robustness.
 <sensor name="imu_sensor" type="imu">
    <gz_frame_id>base_link</gz_frame_id>
    <always_on>1</always_on>
    <update_rate>250</update_rate>
    <imu>
        <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev><bias_mean>0.001</bias_mean><bias_stddev>0.0001</bias_stddev><dynamic_bias_stddev>0.00005</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev><bias_mean>0.001</bias_mean><bias_stddev>0.0001</bias_stddev><dynamic_bias_stddev>0.00005</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev><bias_mean>0.001</bias_mean><bias_stddev>0.0001</bias_stddev><dynamic_bias_stddev>0.00005</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></z>
        </angular_velocity>
        <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev><bias_mean>0.01</bias_mean><bias_stddev>0.001</bias_stddev><dynamic_bias_stddev>0.0001</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev><bias_mean>0.01</bias_mean><bias_stddev>0.001</bias_stddev><dynamic_bias_stddev>0.0001</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev><bias_mean>0.01</bias_mean><bias_stddev>0.001</bias_stddev><dynamic_bias_stddev>0.0001</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></z>
        </linear_acceleration>
    </imu>
</sensor>

<sensor name="magnetometer" type="magnetometer">
    <gz_frame_id>base_link</gz_frame_id>
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <magnetometer>
        <x><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev><bias_mean>0.0001</bias_mean><bias_stddev>0.00001</bias_stddev><dynamic_bias_stddev>0.000001</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></x>
        <y><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev><bias_mean>0.0001</bias_mean><bias_stddev>0.00001</bias_stddev><dynamic_bias_stddev>0.000001</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></y>
        <z><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev><bias_mean>0.0001</bias_mean><bias_stddev>0.00001</bias_stddev><dynamic_bias_stddev>0.000001</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></z>
    </magnetometer>
</sensor>
**
Add Wind Effects (Optional but Recommended):**
<plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
    <horizontal>
        <magnitude>
            <mean>22.0</mean>
            <time_for_rise>10</time_for_rise>
            <sin><amplitude_percent>0.20</amplitude_percent><period>40</period></sin>
        </magnitude>
        <direction>
            <time_for_rise>20</time_for_rise>
            <sin><amplitude>12</amplitude><period>30</period></sin>
        </direction>
    </horizontal>
    <vertical>
        <time_for_rise>8</time_for_rise>
        <noise type="gaussian"><mean>0</mean><stddev>0.5</stddev></noise>
    </vertical>
    <force_approximation_scaling_factor>1.0</force_approximation_scaling_factor>
    <link_name>base_link</link_name>
</plugin>

**Installation**

**Clone the repository:**
cd ~/mahony_ws/src
git clone <your-repo-url>

**Build the package:**
cd ~/mahony_ws
colcon build --packages-select mahony_attitude_control
source install/setup.bash


**Usage**

**1. Start PX4 SITL and Gazebo:**
cd ~/PX4-Autopilot
make px4_sitl gz_rc_cessna


Note: If Gazebo doesn't open, run sudo pkill -f gz && sudo pkill -f gazebo and retry.

**2. Start Micro-XRCE-DDS Agent:**
MicroXRCEAgent udp4 -p 8888


**3. Start ROS-Gazebo Bridge (Required for Ground Truth):**
ros2 run ros_gz_bridge parameter_bridge /model/rc_cessna_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry


**4. Run the Controller:**
source ~/mahony_ws/install/setup.bash
ros2 run mahony_attitude_control mahony_controller


**Data Logging**
Flight data is automatically logged to ~/mahony_logs/. The CSV includes timestamps, raw/filtered sensor data, adaptive gains, and attitude estimation errors compared to ground truth
**ALSO AFTER RUNNING ALL FOUR TERMINAL YOU SHOULD RUN THIS IN PX4 TERMINAL TO MAKE THE PLANE TAKEOFF :**
param set COM_ARM_WO_GPS 1
param set SYS_HAS_MAG 1
param set NAV_DLL_ACT 0
commander arm
commander takeoff

