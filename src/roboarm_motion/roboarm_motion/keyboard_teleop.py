"""Terminal joint jogging for the existing Gazebo trajectory controllers."""
import math
from pathlib import Path
import select
import sys
import termios
import time
import tty
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import xacro


ARM = ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_prism')
GRIPPER = ('joint_5',)
KEYS = {
    'q': ('joint_1', 1), 'a': ('joint_1', -1),
    'w': ('joint_2', 1), 's': ('joint_2', -1),
    'e': ('joint_3', 1), 'd': ('joint_3', -1),
    'r': ('joint_4', 1), 'f': ('joint_4', -1),
    't': ('joint_prism', 1), 'g': ('joint_prism', -1),
    'y': ('joint_5', 1), 'h': ('joint_5', -1),
}
HELP = '''
Keyboard joint controller (focus this terminal)
             increase / decrease
joint_1          Q / A
joint_2          W / S
joint_3          E / D
joint_4          R / F
joint_prism      T / G
joint_5 gripper  Y / H   (joint_6 mimics it)
+ / - : larger / smaller steps     ? : help
Space : hold measured positions    X or Ctrl-C : hold and quit
Each key event jogs one step; holding a key uses terminal key repeat.
Stop other motion clients. Direct joint jogging has no collision checking.
'''


def target_positions(names, measured, joint, delta, limits):
    """Use measured positions, bounding the selected joint to its URDF limits."""
    positions = [measured[name] for name in names]
    index = names.index(joint)
    lower, upper, _ = limits[joint]
    positions[index] = max(lower, min(upper, positions[index] + delta))
    return positions


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('angular_step', 0.04)
        self.declare_parameter('linear_step', 0.005)
        self.declare_parameter('duration', 0.3)
        self.declare_parameter('state_timeout', 1.0)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.duration = float(self.get_parameter('duration').value)
        self.timeout = float(self.get_parameter('state_timeout').value)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.angular_step, self.linear_step, self.duration, self.timeout)):
            raise ValueError('Steps, duration and state_timeout must be finite and positive')
        path = Path(get_package_share_directory('roboarm_urdf_description'))
        robot = ET.fromstring(xacro.process_file(
            str(path / 'urdf' / 'roboArm_urdf.urdf.xacro')).toxml())
        self.limits = {}
        for joint in robot.findall('joint'):
            if joint.get('name') in ARM + GRIPPER:
                limit = joint.find('limit')
                self.limits[joint.get('name')] = tuple(
                    float(limit.get(key)) for key in ('lower', 'upper', 'velocity'))
        if set(self.limits) != set(ARM + GRIPPER):
            raise ValueError('URDF does not describe all controlled joint limits')
        self.measured = {}
        self.received = {}
        self.scale = 1.0
        self.publishers_by_group = {
            ARM: self.create_publisher(
                JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10),
            GRIPPER: self.create_publisher(
                JointTrajectory, '/gripper_controller/joint_trajectory', 10),
        }
        self.create_subscription(JointState, '/joint_states', self.on_state,
                                 qos_profile_sensor_data)

    def on_state(self, msg):
        now = time.monotonic()
        for name, position in zip(msg.name, msg.position):
            if name in self.limits and math.isfinite(position):
                self.measured[name] = position
                self.received[name] = now

    def ready(self, names):
        now = time.monotonic()
        if any(now - self.received.get(name, -math.inf) > self.timeout for name in names):
            print('No fresh /joint_states for this group; command ignored.', flush=True)
            return False
        if not self.publishers_by_group[names].get_subscription_count():
            print('No controller subscriber; command ignored.', flush=True)
            return False
        return True

    def send(self, names, positions):
        # A zero header stamp asks the controller to start upon receipt.
        msg = JointTrajectory()
        msg.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = positions
        # Allow additional time for larger steps according to URDF velocity limits.
        duration = max(self.duration, *(2 * abs(p - self.measured[n]) /
                       self.limits[n][2] for n, p in zip(names, positions)))
        nanoseconds = math.ceil(duration * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
        msg.points = [point]
        self.publishers_by_group[names].publish(msg)

    def hold(self):
        for names in self.publishers_by_group:
            if self.ready(names):
                self.send(names, [self.measured[n] for n in names])

    def key(self, key):
        if key == ' ':
            self.hold()
        elif key in ('+', '=', '-'):
            self.scale = max(0.125, min(4.0, self.scale * (0.5 if key == '-' else 2)))
            print(f'Step scale: {self.scale:g}', flush=True)
        elif key == '?':
            print(HELP, flush=True)
        elif key.lower() in KEYS:
            joint, direction = KEYS[key.lower()]
            names = ARM if joint in ARM else GRIPPER
            if self.ready(names):
                step = self.linear_step if joint == 'joint_prism' else self.angular_step
                positions = target_positions(names, self.measured, joint,
                                             direction * step * self.scale, self.limits)
                self.send(names, positions)
                print(f'{joint}: {positions[names.index(joint)]:.4f} '
                      f'{"m" if joint == "joint_prism" else "rad"}', flush=True)


def main(args=None):
    if not sys.stdin.isatty():
        print('Run keyboard_teleop in an interactive terminal.', file=sys.stderr)
        return
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    settings = termios.tcgetattr(sys.stdin)
    try:
        node = KeyboardTeleop()
        print(HELP, flush=True)
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if select.select([sys.stdin], [], [], 0.04)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'x' or key in ('\x03', ''):
                    break
                node.key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if node is not None:
            if rclpy.ok():
                node.hold()
                # Give the final hold message time to leave before destroying publishers.
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
