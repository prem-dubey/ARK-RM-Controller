
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile , ReliabilityPolicy , HistoryPolicy 
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleAttitudeSetpoint , OffboardControlMode , VehicleCommand
import math


class PIDController:
    def __init__(self , kp , ki , kd , limit = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0
        self.prev_error = 0


    def update(self , error , dt):
        self.integral += error*dt
        self.derivative = (error - self.prev_error)/dt
        self.prev_error = error

        self.output = (
            self.kp * error + 
            self.ki * self.integral +
            self.kd * self.derivative
        )

        if self.limit is not None:
            self.output = max(min(self.output , self.limit ) , -self.limit)

        return self.output
    


class pid(Node):
    def __init__(self):
        super().__init__('pid')

        # qos -----------------------------------
        qos_ekf = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriber ------------------------------
        self.ekf_data = self.create_subscription(
            Float32MultiArray,
            '/ekf_data',
            self.listener_ekf_data,
            qos_ekf
        )

        # Publishers ----------------------------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.pid_data = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )



        # Constants 
        self.roll_r = 0
        self.pitch_r = 0
        self.z_r = 2
        self.z_kp = 1
        self.offboard_started = False
        self.armed = False


        # PID controllers
        self.roll_pid = PIDController(4 , 0 , 0.15 ,0.5 )
        self.pitch_pid = PIDController(4 , 0 , 0.15 , 0.5)
        self.vz_pid = PIDController(3, 0.5,0.2, 0.3)

        # Hover at this thrust value of motor 
        self.thrust_hover = 0.5

        self.last_time = self.get_clock().now()

    def euler_to_quat(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]


    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)


    def listener_ekf_data(self , msg):
        now = self.get_clock().now()
        # ---------- TIME ----------
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        roll, pitch, yaw_rate, z, vz = msg.data

        roll_error = self.roll_r - roll
        pitch_error = self.pitch_r - pitch
        vz_r = (self.z_r - z) * self.z_kp
        vz_error = vz_r - vz
        thrust_correction = self.vz_pid.update(vz_error, dt)

        roll_cmd = self.roll_pid.update(roll_error, dt)
        pitch_cmd = self.pitch_pid.update(pitch_error , dt)
        thrust_cmd = self.thrust_hover + thrust_correction
        thrust_cmd = max(min(thrust_cmd, 0.8), 0.1)  # safety clamp ( see more about this please )

        # ----------- LOG OUTPUT -----------
        self.get_logger().info(
            f"Roll_cmd: {roll_cmd:.3f} | "
            f"Pitch_cmd: {pitch_cmd:.3f} | "
            f"Thrust: {thrust_cmd:.3f}"
        )


        # OffBoard ------------------------------------
        offboard = OffboardControlMode()
        offboard.timestamp = int(now.nanoseconds / 1000)
        offboard.position = False
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = True
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)



        # ARM and OFFBoard for onyl once  
        if not self.armed:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM , param1=1.0)
            self.armed = True
            self.get_logger().info("Vehicle armed")

        if not self.offboard_started:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE , param1=1.0 , param2=6.0)
            self.offboard_started = True
            self.get_logger().info("Offboard mode Activated")

        # Thrust and attitude setpoint 
        att = VehicleAttitudeSetpoint()
        att.timestamp = int(now.nanoseconds / 1000)
        att.q_d = self.euler_to_quat(roll_cmd , pitch_cmd , 0.0)
        att.thrust_body = [0.0 , 0.0 , -thrust_cmd]

        self.pid_data.publish(att)

        # ---------- LOG ----------
        self.get_logger().info(
            f"Roll_cmd: {roll_cmd:.3f} | "
            f"Pitch_cmd: {pitch_cmd:.3f} | "
            f"Thrust: {thrust_cmd:.3f}"
        )



        



def main(args=None):
    rclpy.init(args=args)
    pid_subscriber = pid()
    rclpy.spin(pid_subscriber)
    pid_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()