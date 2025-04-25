import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from rcl_interfaces.msg import SetParametersResult

class QubeController(Node):
    def __init__(self):
        super().__init__('qube_controller')

        # Settpunkt som parameter, kan endres live:
        self.declare_parameter('setpoint', 0.0)
        self.setpoint = self.get_parameter('setpoint').value
        self.add_on_set_parameters_callback(self._on_param_change)

        # Subscriber på /joint_states
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)

        # Publisher til /velocity_controller/commands
        self.pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # PID‑parametre
        self.kp = 15.0
        self.ki = 0.2
        self.kd = 1.0
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = self.get_clock().now()

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'setpoint' and p.type_ == p.Type.DOUBLE:
                self.get_logger().info(f"New setpoint: {p.value}")
                self.setpoint = p.value
        return SetParametersResult(successful=True)

    def joint_cb(self, msg: JointState):
        # Gjelder første joint (motor_joint)
        if not msg.position or not msg.velocity:
            return

        pos = msg.position[0]
        vel = msg.velocity[0]
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # PID‐beregning på feil i posisjon
        error = self.setpoint - pos
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt

        cmd = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Bygg Float64MultiArray
        ma = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = 'joint_velocity'
        dim.size = 1
        dim.stride = 1
        ma.layout.dim = [dim]
        ma.layout.data_offset = 0
        ma.data = [cmd]

        # Publiser hastighets‑pådrag
        self.pub.publish(ma)

        # Lagring til neste runde
        self._prev_error = error
        self._prev_time = now

def main(args=None):
    rclpy.init(args=args)
    node = QubeController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()