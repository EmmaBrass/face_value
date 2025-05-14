from robot_control import robot_operation

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

if __name__ == '__main__':
    # Demonstrate usage
    example_movement()
