"""ROS 2 node: MPU6050 on the Pi's I2C bus -> sensor_msgs/Imu (gyro + accel, no orientation).

Publishing starts once a stationary calibration has succeeded (robot still for about 2 s at
startup). Calling ~/calibrate (std_srvs/Trigger) while the robot stands still, for example at a
waypoint, re-measures the gyro bias; publishing continues with the old bias meanwhile.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from .calibration import NotStill, apply, calibrate
from .mpu6050 import KNOWN_IDS, MPU6050, explain_io_error, open_bus

RETRY_S = 1.0


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu")
        for name, default in [("i2c_bus", 1), ("address", 0x68), ("frame_id", "imu_link"),
                              ("topic", "imu/data_raw"), ("rate_hz", 100.0),
                              ("gyro_range_dps", 250), ("accel_range_g", 2), ("dlpf", 3),
                              ("calibration_samples", 200), ("max_still_gyro_std", 0.02)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame_id = p("frame_id")
        self.n_cal = int(p("calibration_samples"))
        self.max_std = float(p("max_still_gyro_std"))
        self.dev = MPU6050(open_bus(int(p("i2c_bus"))), int(p("address")), int(p("gyro_range_dps")),
                           int(p("accel_range_g")), int(p("dlpf")), float(p("rate_hz")))
        self.cal = None
        self.cal_buf = []
        self.calibrating = True
        self.connected = False
        self.next_retry = 0.0
        self.pub = self.create_publisher(Imu, p("topic"), qos_profile_sensor_data)
        self.create_service(Trigger, "~/calibrate", self._on_calibrate)
        self.create_timer(1.0 / float(p("rate_hz")), self._tick)
        self.get_logger().info("keep the robot still: calibrating the gyro")

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _connect(self):
        if self._now_s() < self.next_retry:
            return
        try:
            chip = self.dev.who_am_i()
            if chip not in KNOWN_IDS:
                self.get_logger().warn(f"unexpected WHO_AM_I 0x{chip:02x}; trying anyway")
            self.dev.configure()
            self.connected = True
            self.get_logger().info(f"{KNOWN_IDS.get(chip, 'unknown chip')} at "
                                   f"0x{self.dev.address:02x}, {self.dev.chip_rate_hz:.0f} Hz")
        except OSError as e:
            self.next_retry = self._now_s() + RETRY_S
            self.get_logger().error(explain_io_error(e, self.dev.address),
                                    throttle_duration_sec=10.0)

    def _on_calibrate(self, _req, res):
        self.cal_buf, self.calibrating = [], True
        res.success = True
        res.message = f"collecting {self.n_cal} samples; keep the robot still"
        return res

    def _tick(self):
        if not self.connected:
            self._connect()
            return
        stamp = self.get_clock().now().to_msg()
        try:
            s = self.dev.read()
        except OSError as e:
            self.connected = False
            self.get_logger().error("read failed: " + explain_io_error(e, self.dev.address))
            return
        if self.calibrating:
            self._calibration_step(s)
        if self.cal is not None:
            self.pub.publish(self._message(apply(s, self.cal), stamp))

    def _calibration_step(self, s):
        self.cal_buf.append(s)
        if len(self.cal_buf) < self.n_cal:
            return
        try:
            self.cal = calibrate(self.cal_buf, self.max_std)
            self.calibrating = False
            b = [round(x * 57.2958, 3) for x in self.cal.gyro_bias]
            self.get_logger().info(f"calibrated: gyro bias {b} deg/s, "
                                   f"|g| {self.cal.gravity:.2f} m/s^2")
        except NotStill as e:
            self.get_logger().warn(f"calibration retry: {e}")
        self.cal_buf = []

    def _message(self, s, stamp):
        m = Imu()
        m.header.stamp, m.header.frame_id = stamp, self.frame_id
        m.orientation_covariance[0] = -1.0          # no orientation estimate in this message
        (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z) = s.gyro
        (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z) = s.accel
        for i in range(3):
            m.angular_velocity_covariance[4 * i] = self.cal.gyro_var[i]
            m.linear_acceleration_covariance[4 * i] = self.cal.accel_var[i]
        return m


def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
