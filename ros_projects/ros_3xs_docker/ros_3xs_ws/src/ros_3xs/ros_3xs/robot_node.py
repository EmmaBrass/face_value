import rclpy
from rclpy.node import Node
from robot_control import robot_operation

class RobotNode(Node):

    def __init__(self):
        super().__init__('robot_node')

    @robot_operation(mode='simulation')
    def example_movement(robot):
        """
        Example robot movement function.
        
        :param robot: RobotController instance
        """
        # Perform relative movement
        robot.controller.move_base_relative(
            20.0, 20.0, 20.0,  # x, y, z movement
            0.0, 0.0, 0.0,     # rotational movement
            0.0, 0.0, 0.0,     # tool coordinates
            10.0,              # speed
            wait_move_finished=True,
            move_finished_timeout=1000
        )

def main(args=None):
    rclpy.init(args=args)

    robot_node = RobotNode()

    rclpy.spin(robot_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    