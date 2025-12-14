#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import os
from datetime import datetime
from collections import deque

from px4_msgs.msg import SensorCombined, VehicleAttitude, ActuatorMotors, Wind, VehicleMagnetometer
from px4_msgs.msg import OffboardControlMode, VehicleCommand
from nav_msgs.msg import Odometry

class AdvancedMahonyController(Node):
    def __init__(self):
        super().__init__('advanced_mahony_controller')
        
        self.drone_model_name = 'rc_cessna_0'
        self.actual_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        
        self.KpD = 1.5
        self.KiD = 0.05
        self.Kpmin = 0.1
        self.Kimin = 0.01
        
        self.w_a = 0.6
        self.w_b = 0.3
        self.w_g = 1
        
        self.athr = 9.8
        self.wth = 2.0
        self.mag_ref = 50e-6
        self.gyro_max = 5.0
        self.accel_max = 2.0
        self.omega_max = 5.0
        self.sigma_max = 0.5
        self.omega_dot_max = 10.0
        
        self.dt = 0.004
        
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro_bias = np.zeros(3)
        self.integral_err = np.zeros(3)
        self.mag_bias = np.zeros(3)
        
        self.mag_declination = 0.0
        
        self.accel_prev = np.zeros(3)
        self.accel_prev2 = np.zeros(3)
        self.accel_filt_prev = np.zeros(3)
        self.accel_filt_prev2 = np.zeros(3)
        self.gyro_filt_prev = np.zeros(3)
        self.mag_filt_prev = np.zeros(3)
        
        self.gyro_history = deque(maxlen=100)
        self.gyro_variance = np.zeros(3)
        self.omega_prev = np.zeros(3)
        
        self.accel_b = np.array([0.0067, 0.0134, 0.0067])
        self.accel_a = np.array([1.0, -1.1429, 0.4128])
        self.gyro_alpha = 0.1
        self.mag_alpha = 0.01
        self.mag_scale = np.eye(3)
        
        self.initialized = False
        self.mag_data_received = False
        self.calibration_samples = 0
        self.calibration_target = 500
        self.gyro_bias_sum = np.zeros(3)
        self.mag_bias_sum = np.zeros(3)
        self.alignment_done = False
        
        self.offboard_setpoint_counter = 0
        self.offboard_enabled = False
        
        self.log_data = True
        self.log_counter = 0
        self.log_dt = 0.1
        self.last_log_time = 0.0
        self.log_file = None
        self.setup_data_logging()
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sensor_sub = self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined', 
            self.sensor_callback, qos_profile)
            
        self.mag_sub = self.create_subscription(
            VehicleMagnetometer, '/fmu/out/vehicle_magnetometer',
            self.magnetometer_callback, qos_profile)
        
        self.wind_sub = self.create_subscription(
            Wind, '/fmu/out/wind', 
            self.wind_callback, qos_profile)
        
        self.odom_sub = self.create_subscription(
            Odometry, f'/model/{self.drone_model_name}/odometry',
            self.odom_callback, 10)
        
        self.attitude_pub = self.create_publisher(
            VehicleAttitude, '/fmu/in/vehicle_attitude', qos_profile)
        
        self.actuator_pub = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0
        self.throttle = 0.55
        self.roll_kp = 0.3
        self.pitch_kp = 0.15
        self.yaw_kp = 0.3
        
        self.wind_speed = np.zeros(3)
        self.latest_mag_data = np.zeros(3)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Controller Initialized')
    
    def setup_data_logging(self):
        if not self.log_data:
            return
        log_dir = os.path.expanduser("~/mahony_logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"mahony_flight_data_{timestamp}.csv"
        log_path = os.path.join(log_dir, log_filename)
        self.log_file = open(log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        header = [
            'timestamp', 'dt',
            'gyro_x_raw', 'gyro_y_raw', 'gyro_z_raw',
            'accel_x_raw', 'accel_y_raw', 'accel_z_raw',
            'mag_x_raw', 'mag_y_raw', 'mag_z_raw',
            'gyro_x_filt', 'gyro_y_filt', 'gyro_z_filt',
            'accel_x_filt', 'accel_y_filt', 'accel_z_filt',
            'mag_x_filt', 'mag_y_filt', 'mag_z_filt',
            'accel_confidence', 'mag_confidence', 'gyro_confidence', 'combined_confidence',
            'kp_adaptive', 'ki_adaptive',
            'roll_sensor', 'pitch_sensor', 'yaw_sensor',
            'roll_actual', 'pitch_actual', 'yaw_actual',
            'roll_error', 'pitch_error', 'yaw_error',
            'q_w', 'q_x', 'q_y', 'q_z'
        ]
        self.csv_writer.writerow(header)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        q_ned = [q.w, q.y, q.x, -q.z]
        r = R.from_quat([q_ned[1], q_ned[2], q_ned[3], q_ned[0]])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        self.actual_attitude = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    def magnetometer_callback(self, msg):
        self.latest_mag_data = np.array(msg.magnetometer_ga) * 1e-4
        if not self.mag_data_received:
            self.mag_data_received = True

    def wind_callback(self, msg):
        self.wind_speed = np.array([msg.windspeed_north, msg.windspeed_east, 0.0])

    def apply_accel_lpf(self, accel_raw):
        accel_filt = (self.accel_b[0] * accel_raw +
                     self.accel_b[1] * self.accel_prev +
                     self.accel_b[2] * self.accel_prev2 -
                     self.accel_a[1] * self.accel_filt_prev -
                     self.accel_a[2] * self.accel_filt_prev2)
        
        self.accel_prev2, self.accel_prev = self.accel_prev.copy(), accel_raw.copy()
        self.accel_filt_prev2, self.accel_filt_prev = self.accel_filt_prev.copy(), accel_filt.copy()
        return accel_filt

    def apply_gyro_lpf(self, gyro_raw):
        gyro_filt = (1.0 - self.gyro_alpha) * self.gyro_filt_prev + self.gyro_alpha * gyro_raw
        self.gyro_filt_prev = gyro_filt.copy()
        return gyro_filt

    def apply_mag_lpf(self, mag_raw):
        mag_filt = (1.0 - self.mag_alpha) * self.mag_filt_prev + self.mag_alpha * mag_raw
        self.mag_filt_prev = mag_filt.copy()
        return mag_filt

    def calculate_accel_confidence(self, accel, gyro):
        accel_mag = np.linalg.norm(accel)
        gyro_mag = np.linalg.norm(gyro)
        gravity_factor = np.clip(1.0 - abs(accel_mag - 9.8) / 5.5, 0.0, 1.0)
        angular_factor = np.clip(1.0 - gyro_mag / self.wth, 0.0, 1.0)
        return gravity_factor * angular_factor

    def calculate_mag_confidence(self, mag, gyro, gravity_error_norm):
        mag_mag = np.linalg.norm(mag)
        gyro_mag = np.linalg.norm(gyro)
        mag_factor = np.clip(1.0 - abs(mag_mag - self.mag_ref) / (self.mag_ref * 2.0), 0.0, 1.0)
        gyro_factor = np.clip(1.0 - gyro_mag / self.gyro_max, 0.0, 1.0)
        accel_factor = np.clip(1.0 - gravity_error_norm / 0.5, 0.0, 1.0)
        return mag_factor * gyro_factor * accel_factor

    def calculate_gyro_confidence(self, gyro):
        gyro_mag = np.linalg.norm(gyro)
        self.gyro_history.append(gyro.copy())
        if len(self.gyro_history) >= 10:
            self.gyro_variance = np.var(np.array(self.gyro_history), axis=0)
        
        omega_dot = (gyro - self.omega_prev) / self.dt
        self.omega_prev = gyro.copy()
        omega_dot_mag = np.linalg.norm(omega_dot)
        
        omega_factor = np.clip(1.0 - gyro_mag / self.omega_max, 0.0, 1.0)
        var_factor = np.clip(1.0 - np.linalg.norm(self.gyro_variance) / self.sigma_max, 0.0, 1.0)
        accel_factor = np.clip(1.0 - omega_dot_mag / self.omega_dot_max, 0.0, 1.0)
        return omega_factor * var_factor * accel_factor

    def align_initial_attitude(self, accel_filt, mag_filt):
        if self.alignment_done:
            return
        
        if abs(self.actual_attitude['roll']) < 0.01 and abs(self.actual_attitude['pitch']) < 0.01 and abs(self.actual_attitude['yaw']) < 0.01:
            return
        
        roll_gt = np.radians(self.actual_attitude['roll'])
        pitch_gt = np.radians(self.actual_attitude['pitch'])
        yaw_gt = np.radians(self.actual_attitude['yaw'])
        
        r_gt = R.from_euler('xyz', [roll_gt, pitch_gt, yaw_gt])
        q_gt = r_gt.as_quat()
        
        self.q = np.array([q_gt[3], q_gt[0], q_gt[1], q_gt[2]])
        
        accel_norm = np.linalg.norm(accel_filt)
        if accel_norm < 1e-6:
            return
        g = accel_filt / accel_norm
        
        mag_cal = self.mag_scale @ (mag_filt - self.mag_bias)
        mag_norm = np.linalg.norm(mag_cal)
        if mag_norm < 1e-6:
            return
        Bb = mag_cal / mag_norm
        
        Hb = Bb - np.dot(Bb, g) * g
        Hb_norm = np.linalg.norm(Hb)
        if Hb_norm < 1e-6:
            return
        Hb = Hb / Hb_norm
        
        mag_indicated_yaw = np.arctan2(Hb[1], Hb[0])
        
        self.mag_declination = yaw_gt - mag_indicated_yaw
        
        while self.mag_declination > np.pi:
            self.mag_declination -= 2 * np.pi
        while self.mag_declination < -np.pi:
            self.mag_declination += 2 * np.pi
        
        self.alignment_done = True
        print(f"Alignment Complete. Declination: {np.degrees(self.mag_declination):.2f} deg")

    def mahony_update(self, gyro_filt, accel_filt, mag_filt):
        accel_norm = np.linalg.norm(accel_filt)
        if accel_norm < 1e-6:
            return
        g = accel_filt / accel_norm
        
        mag_cal = self.mag_scale @ (mag_filt - self.mag_bias)
        mag_norm = np.linalg.norm(mag_cal)
        if mag_norm < 1e-6:
            return
        Bb = mag_cal / mag_norm
        
        Hb = Bb - np.dot(Bb, g) * g
        Hb_norm = np.linalg.norm(Hb)
        if Hb_norm < 1e-6:
            return
        Hb = Hb / Hb_norm
        
        q0, q1, q2, q3 = self.q
        Vg = np.array([
            2*(q1*q3 - q0*q2),
            2*(q0*q1 + q2*q3),
            q0**2 - q1**2 - q2**2 + q3**2
        ])
        
        Vn = np.array([
            q0**2 + q1**2 - q2**2 - q3**2,
            2*(q1*q2 + q0*q3),
            2*(q1*q3 - q0*q2)
        ])
        
        cos_dec = np.cos(self.mag_declination)
        sin_dec = np.sin(self.mag_declination)
        
        Vn_corrected = np.array([
            Vn[0] * cos_dec + Vn[1] * sin_dec,
            -Vn[0] * sin_dec + Vn[1] * cos_dec,
            Vn[2]
        ])
        
        Vn_corrected_norm = np.linalg.norm(Vn_corrected)
        if Vn_corrected_norm > 1e-12:
            Vn_corrected = Vn_corrected / Vn_corrected_norm
        
        gravity_err = np.cross(Vg, g)
        mag_err = np.cross(Vn_corrected, Hb)
        
        Ra = self.calculate_accel_confidence(accel_filt, gyro_filt)
        gravity_error_norm = np.linalg.norm(gravity_err)
        Rb = self.calculate_mag_confidence(mag_filt, gyro_filt, gravity_error_norm)
        Rg = self.calculate_gyro_confidence(gyro_filt)
        
        r = np.clip((self.w_a * Ra + self.w_b * Rb + self.w_g * Rg) / (self.w_a + self.w_b + self.w_g), 0.0, 1.0)
        
        Kp = self.KpD * (self.Kpmin + (1 - self.Kpmin) * r)
        Ki = self.KiD * (self.Kimin + (1 - self.Kimin) * r)
        
        total_err = self.w_a * Ra * gravity_err + self.w_b * Rb * mag_err
        
        self.integral_err += Ki * total_err * self.dt
        
        omega_corr = gyro_filt - self.gyro_bias + Kp * total_err + self.integral_err
        
        phi = 0.5 * omega_corr * self.dt
        phi_norm = np.linalg.norm(phi)
        
        if phi_norm > 1e-12:
            s_phi = np.sin(phi_norm) / phi_norm
            q_delta = np.array([np.cos(phi_norm), s_phi*phi[0], s_phi*phi[1], s_phi*phi[2]])
        else:
            q_delta = np.array([1.0, phi[0], phi[1], phi[2]])
        
        qw, qx, qy, qz = self.q
        dw, dx, dy, dz = q_delta
        q_new = np.array([
            qw*dw - qx*dx - qy*dy - qz*dz,
            qw*dx + qx*dw + qy*dz - qz*dy,
            qw*dy - qx*dz + qy*dw + qz*dx,
            qw*dz + qx*dy - qy*dx + qz*dw
        ])
        
        self.q = q_new / np.linalg.norm(q_new)
        
        self.last_confidence = {'Ra': Ra, 'Rb': Rb, 'Rg': Rg, 'combined': r}
        self.last_gains = {'Kp': Kp, 'Ki': Ki}

    def sensor_callback(self, msg):
        if not self.mag_data_received:
            return
        
        gyro_raw = np.array(msg.gyro_rad)
        accel_raw = np.array(msg.accelerometer_m_s2) * 9.8665
        mag_raw = self.latest_mag_data
        
        if not self.initialized:
            self.calibration_samples += 1
            self.gyro_bias_sum += gyro_raw
            self.mag_bias_sum += mag_raw
            
            if self.calibration_samples >= self.calibration_target:
                self.gyro_bias = self.gyro_bias_sum / self.calibration_samples
                self.mag_bias = self.mag_bias_sum / self.calibration_samples
                self.initialized = True
                print(f"Sensor Calibration Complete")
            return
        
        accel_filt = self.apply_accel_lpf(accel_raw)
        gyro_filt = self.apply_gyro_lpf(gyro_raw)
        mag_filt = self.apply_mag_lpf(mag_raw)
        
        if not self.alignment_done:
            self.align_initial_attitude(accel_filt, mag_filt)
            if not self.alignment_done:
                return
        
        self.mahony_update(gyro_filt, accel_filt, mag_filt)
        
        attitude_msg = VehicleAttitude()
        attitude_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        attitude_msg.q = [self.q[0], self.q[1], self.q[2], self.q[3]]
        self.attitude_pub.publish(attitude_msg)
        
        self.last_sensor_data = {
            'gyro_raw': gyro_raw, 'accel_raw': accel_raw, 'mag_raw': mag_raw,
            'gyro_filt': gyro_filt, 'accel_filt': accel_filt, 'mag_filt': mag_filt,
            'timestamp': msg.timestamp
        }

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arming...')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('Engaging Offboard Mode')

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def control_loop(self):
        if not self.initialized or not self.alignment_done or not hasattr(self, 'last_sensor_data'):
            return
        
        self.publish_offboard_control_mode()
        
        self.offboard_setpoint_counter += 1
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_enabled = True
        
        r = R.from_quat([self.q[1], self.q[2], self.q[3], self.q[0]])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        
        aileron_cmd = np.clip((self.desired_roll - roll) * self.roll_kp, -1.0, 1.0)
        elevator_cmd = np.clip((self.desired_pitch - pitch) * self.pitch_kp, -1.0, 1.0)
        rudder_cmd = np.clip((self.desired_yaw - yaw) * self.yaw_kp, -1.0, 1.0)
        
        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        actuator_msg.control = [self.throttle, aileron_cmd, elevator_cmd, rudder_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.actuator_pub.publish(actuator_msg)
        
        if self.log_data:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.last_log_time) >= self.log_dt:
                self.log_flight_data(roll, pitch, yaw)
                self.last_log_time = current_time
        
        if not hasattr(self, 'debug_count'):
            self.debug_count = 0
        self.debug_count += 1
        
        if self.debug_count % 250 == 0:
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)
            
            roll_err = self.normalize_angle(roll_deg - self.actual_attitude["roll"])
            pitch_err = self.normalize_angle(pitch_deg - self.actual_attitude["pitch"])
            yaw_err = self.normalize_angle(yaw_deg - self.actual_attitude["yaw"])
            
            conf = getattr(self, 'last_confidence', {'Ra': 0, 'Rb': 0, 'Rg': 0, 'combined': 0})
            
            print(f"Time: {self.offboard_setpoint_counter * self.dt:.1f}s")
            print(f"Attitude (Deg) - Roll: {roll_deg:.2f}, Pitch: {pitch_deg:.2f}, Yaw: {yaw_deg:.2f}")
            print(f"Error (Deg)    - Roll: {roll_err:.2f}, Pitch: {pitch_err:.2f}, Yaw: {yaw_err:.2f}")
            print(f"Confidence     - Acc: {conf['Ra']:.2f}, Mag: {conf['Rb']:.2f}, Gyr: {conf['Rg']:.2f}, All: {conf['combined']:.2f}")
            print("-" * 30)

    def log_flight_data(self, roll, pitch, yaw):
        if not self.log_file:
            return
        
        ts = self.get_clock().now().nanoseconds / 1e9
        sensor = self.last_sensor_data
        conf = getattr(self, 'last_confidence', {'Ra': 0, 'Rb': 0, 'Rg': 0, 'combined': 0})
        gains = getattr(self, 'last_gains', {'Kp': 0, 'Ki': 0})
        
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        
        roll_err = self.normalize_angle(roll_deg - self.actual_attitude['roll'])
        pitch_err = self.normalize_angle(pitch_deg - self.actual_attitude['pitch'])
        yaw_err = self.normalize_angle(yaw_deg - self.actual_attitude['yaw'])
        
        row = [
            ts, self.dt,
            *sensor['gyro_raw'], *sensor['accel_raw'], *sensor['mag_raw'],
            *sensor['gyro_filt'], *sensor['accel_filt'], *sensor['mag_filt'],
            conf['Ra'], conf['Rb'], conf['Rg'], conf['combined'],
            gains['Kp'], gains['Ki'],
            roll_deg, pitch_deg, yaw_deg,
            self.actual_attitude['roll'], self.actual_attitude['pitch'], self.actual_attitude['yaw'],
            roll_err, pitch_err, yaw_err,
            *self.q
        ]
        
        self.csv_writer.writerow(row)
        
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.log_file.flush()

    def __del__(self):
        if self.log_file:
            self.log_file.close()

def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedMahonyController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()