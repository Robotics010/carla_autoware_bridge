from autoware_auto_vehicle_msgs.msg import Engage
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from autoware_auto_system_msgs.msg import AutowareState
import time


class GoalSender(Node):
    def __init__(self):
        super().__init__("goal_sender")

        self._state_subscriber = self.create_subscription(
            AutowareState, '/autoware/state', self._state_callback, 1
        )

        self._goal_pose_publisher = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 1)
        self._goal_poses = self._get_goal_poses()
        self._pose_index = 0

        self._engage_publisher = self.create_publisher(
            Engage, '/autoware/engage', 1)
        self._engaged = False
        self._arrived_goal = False

        self._publish_next_pose()

    def _get_goal_poses(self):
        goal_poses = []

        # little_square
        # right
        goal_poses.append(self._generate_pose_message(
            (154.23333740234375, -26.385265350341797, 0.0),
            (0.0, 0.0, -0.7071067811865475, 0.7071067811865476)))
        # # bottom
        goal_poses.append(self._generate_pose_message(
            (129.13331604003906, -55.485260009765625, 0.0),
            (0.0, 0.0, 1.0, 6.123233995736766e-17)))
        # # left
        goal_poses.append(self._generate_pose_message(
            (92.53333282470703, -27.98528289794922, 0.0),
            (0.0, 0.0, 0.7071067811865475, 0.7071067811865476)))
        # # up (start)
        goal_poses.append(self._generate_pose_message(
            (124.83332824707031, -1.9853160381317139, 0.0),
            (0.0, 0.0, 0.0, 1.0)))
        
        # middle_square
        # right
        # goal_poses.append(self._generate_pose_message(
        #     (335.06072998046875, -14.563956260681152, 0.0),
        #     (0.0, 0.0, -0.7091711412851084, 0.7050363766277431)))
        # # bottom
        # goal_poses.append(self._generate_pose_message(
        #     (321.2488098144531, -195.33975219726562, 0.0),
        #     (0.0, 0.0, 0.999998870588642, 0.0015029376036528794)))
        # # left
        # goal_poses.append(self._generate_pose_message(
        #     (92.4143295288086, -182.85816955566406, 0.0),
        #     (0.0, 0.0, 0.7064671280017225, 0.7077458562598568)))
        # # up (start)
        # goal_poses.append(self._generate_pose_message(
        #     (124.83332824707031, -1.9853160381317139, 0.0),
        #     (0.0, 0.0, 0.0, 1.0)))

        # large_square
        # right
        # goal_poses.append(self._generate_pose_message(
        #     (392.4088134765625, -13.688782691955566, 0.0),
        #     (0.0, 0.0, -0.7050054866000136, 0.7092018498734179)))
        # # bottom
        # goal_poses.append(self._generate_pose_message(
        #     (378.9555969238281, -326.68255615234375, 0.0),
        #     (0.0, 0.0, 0.9999883691446513, 0.004823025546321005)))
        # # left
        # goal_poses.append(self._generate_pose_message(
        #     (1.948915958404541, -316.89263916015625, 0.0),
        #     (0.0, 0.0, 0.7045402054436387, 0.7096640746955107)))
        # # up (start)
        # goal_poses.append(self._generate_pose_message(
        #     (124.83332824707031, -1.9853160381317139, 0.0),
        #     (0.0, 0.0, 0.0, 1.0)))

        return goal_poses

    def _generate_pose_message(self, position, orientation):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        x, y, z = position
        qx, qy, qz, qw = orientation
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _state_callback(self, state_msg):
        self._state = state_msg.state

        # state: 2 # waiting_for_route
        if self._state == AutowareState.ARRIVED_GOAL and not self._arrived_goal:
            self._publish_next_pose()
            self._arrived_goal = True
            self._engaged = False
        elif self._state == AutowareState.WAITING_FOR_ENGAGE and not self._engaged:
            self._publish_engage()
            self._engaged = True
            self._arrived_goal = False

    def _publish_next_pose(self):
        goal_pose_msg = self._get_next_goal_pose()
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self._goal_pose_publisher.publish(goal_pose_msg)
        print(f'publishing goal_pose:\n{goal_pose_msg}')

    def _get_next_goal_pose(self):
        next_index = self._pose_index % len(self._goal_poses)
        self._pose_index += 1
        return self._goal_poses[next_index]

    def _publish_engage(self):
        engage_msg = self._generate_engage_message()
        self._engage_publisher.publish(engage_msg)
        print(f'publishing engage:\n{engage_msg}')

    def _generate_engage_message(self):
        engage_msg = Engage()
        engage_msg.stamp = self.get_clock().now().to_msg()
        engage_msg.engage = True
        return engage_msg

def main(args=None):
    rclpy.init(args=args)
    steer_commander = GoalSender()
    
    rclpy.spin(steer_commander)
    
    steer_commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()