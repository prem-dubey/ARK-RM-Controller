import rclpy
from rclpy.node import Node
from std_msgs.msg import String , Float32MultiArray
import std_msgs
from px4_msgs.msg import SensorCombined , VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy , DurabilityPolicy
from .ekf_cal import ImuData , ekf_step
import numpy as np




class ekfnode(Node):
    def __init__(self):
        super().__init__('ekf')

        # qos --------------------------------------------

        qos_imu = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        qos_vehicle = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )


        ekf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subsscribers --------------------------------------

        self.imu_sensor_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.listener_imu,
            qos_imu
        )

        self.altimeter_sensor_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.listener_altimeter,
            qos_vehicle
        )

        # Publishers --------------------------------------------

        self.ekf_data = self.create_publisher(
            Float32MultiArray,
            '/ekf_data',
            ekf_qos
        )


        # States ---------------------------------------------
        self.latest_imu = None
        self.latest_alt = None  # (z, vz, reset_counter)
        self.p = np.diag([
                (0.0873)**2,   # roll variance
                (0.0873)**2,   # pitch variance
                (0.349)**2,    # yaw rate variance
                (2.0)**2,      # altitude variance (m^2)
                (1.0)**2       # vertical velocity variance (m^2/s^2)
            ])
        self.x = np.array([
                0.0,          # roll (rad)
                0.0,          # pitch (rad)
                0.0,          # yaw rate (rad/s)
                0.0,          # altitude (m)
                0.0           # vertical velocity (m/s)
            ])

        self.counter = 0

    
    def listener_imu(self, msg):
        # Creating imu data-------------------------------
        self.counter += 1
        self.latest_imu = ImuData(
            accel = (
            float(msg.accelerometer_m_s2[0]),
            float(msg.accelerometer_m_s2[1]),
            float(msg.accelerometer_m_s2[2]),
        ),
        gyro = (
            float(msg.gyro_rad[0]),
            float(msg.gyro_rad[1]),
            float(msg.gyro_rad[2]),
        ),
        accel_dt=msg.accelerometer_integral_dt * 1e-6,
        gyro_dt=msg.gyro_integral_dt * 1e-6,
        timestamp=msg.timestamp * 1e-6,
        accelerometer_clipping=msg.accelerometer_clipping,
        gyro_clipping=msg.gyro_clipping
        )

        # # Printing input ------------------------------
        # if  self.counter%100 == 0:
        #     self.get_logger().info(
        #         "\n----- EKF INPUT -----\n"
        #         f"Accel (m/s^2): {self.latest_imu.accel}\n"
        #         f"Gyro  (rad/s): {self.latest_imu.gyro}\n"
        #         f"Accel dt     : {self.latest_imu.accel_dt:.6f}\n"
        #         f"Gyro  dt     : {self.latest_imu.gyro_dt:.6f}\n"
        #         f"Timestamp    : {self.latest_imu.timestamp:.6f}\n"
        #         f"Altimeter    : {self.latest_alt}\n"
        #         "---------------------"
        #     )

        self.x , self.p = ekf_step(self.x , self.p , self.latest_imu , self.latest_alt )

        ekf_msg = Float32MultiArray()
        ekf_msg.data = [
            float(self.x[0]),
            float(self.x[1]),
            float(self.x[2]),
            float(self.x[3]),
            float(self.x[4])
        ]
        self.ekf_data.publish(ekf_msg)

        # if self.counter%100 == 0:
        #     self.get_logger().info(
        #         "\n===== EKF OUTPUT =====\n"
        #         f"Roll  (rad): {self.x[0]: .4f}\n"
        #         f"Pitch (rad): {self.x[1]: .4f}\n"
        #         f"Yaw r (rad/s): {self.x[2]: .4f}\n"
        #         f"Z     (m): {self.x[3]: .4f}\n"
        #         f"Vz    (m/s): {self.x[4]: .4f}\n"
        #         f"Var(z): {self.p[3,3]: .4f}\n"
        #         "======================"
        #     )

        self.latest_alt = None
    

    def listener_altimeter(self , msg):

        if not msg.z_valid:
            return
        
        self.latest_alt = (
            msg.z,
            msg.vz,
            msg.z_reset_counter
        )

        # self.get_logger().info(
        #     f"Altimeter : {self.latest_alt} \n"
        # )


        
# Till now everyhting is up and working


def main(args=None):
    rclpy.init(args=args)

    sensor_subscriber = ekfnode()

    rclpy.spin(sensor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



