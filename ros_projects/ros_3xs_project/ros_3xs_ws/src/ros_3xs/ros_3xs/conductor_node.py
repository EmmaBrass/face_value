import rclpy
from rclpy.node import Node

import face_recognition
from conductor_node import ConductorStateMachine

class ConductorNode(Node):

    def __init__(self):
        super().__init__('conductor_node')

        self.state_machine = ConductorStateMachine()
        self.prev_state_complete = False

        # Initialise publishers
        self.conductor_publisher = self.create_publisher(ConductorState, 'conductor_state', 10)

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

    def timer_callback(self):
        """
        Checks the state of the state machine every timer_period,
        and based on the state, publishes and/or runs a function to move
        it on to the next state.
        Publishes to system_state topic.
        and publishes to other topics if in the appropriate state.
        """
        self.get_logger().info(f'Current system state: {self.state_machine.state}')
        # Publish the current state
        self.conductor_publisher.publish(str(self.state_machine.state)) # TODO need proper message
        # If prev state processes completed, tick the state forward.
        if self.prev_state_complete == True:
            self.prev_state_complete = False
            # Run a function for the given state, to tick the state machine forwards by one.
            if self.state_machine.state == 'startup':
                self.startup()
            elif self.state_machine.state == 'face_choosing':
                self.face_choosing()
            elif self.state_machine.state == 'face_chosen':
                self.face_chosen()
            elif self.state_machine.state == 'face_move_1_2':
                self.face_move_1_2()
            elif self.state_machine.state == 'face_edit':
                self.face_edit()
            elif self.state_machine.state == 'face_edit_done':
                self.face_edit_done()
            elif self.state_machine.state == 'face_move_2_3':
                self.face_move_2_3()

        # TODO have processing here! e.g. if we are about to move into face_edit, but the move is not done yet, 
        # can be preemptively preparing for the edit my choosing edit types and path vectors etc, to avoid delays.

    def startup(self):
        """
        Whatever processes happen in the startup state.
        """
        pass

    def face_choosing(self):
        """
        Whatever processes happen in the face_choosing state.
        """
        pass

    def face_chosen(self):
        """
        Whatever processes happen in the face_chosen state.
        """
        pass

    def face_move_1_2(self):
        """
        Whatever processes happen in the face_move_1_2 state.
        """
        # potent
        pass

    def face_edit(self):
        """
        Whatever processes happen in the face_edit state.
        """
        # An edit happens to the face.
        # With, e.g., 3 arm motions (could reduce or increase this number based on how long each one takes).
        # Within this method, choose the edit types and generate the path vector.
        # Then, path vector and edit type needs to get passed to TD, and robot, and LLM !

        # Generate a bezier curve that fits well within the size of screen 2.

        # Extract an array of coordinates, for sending to the robot.

        # Extract an arroy of coordinates in whatever format is needed for TD.

        

        pass

    def face_edit_done(self):
        """
        Whatever processes happen in the face_edit_done state.
        """
        pass

    def face_move_2_3(self):
        """
        Whatever processes happen in the face_move_2_3 state.
        """
        pass

def main(args=None):
    rclpy.init(args=args)

    conductor_node = ConductorNode()

    rclpy.spin(conductor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    conductor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


