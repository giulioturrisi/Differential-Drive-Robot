import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Planner_A_star(Node):

    def __init__(self):
        super().__init__('Planner_A_star')
        self.publisher_path = self.create_publisher(Path, 'path', 1)

        self.subscription_goal = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.subscription_tf = self.create_subscription(PoseStamped,'tf',self.tf_callback,1)
        self.subscription_map = self.create_subscription(PoseStamped,'map',self.map_callback,1)




def main(args=None):
    rclpy.init(args=args)

    planner_node = Planner_A_star()

    rclpy.spin(planner_node)


    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()