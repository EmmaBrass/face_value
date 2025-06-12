from transitions import Machine

# State machine for the installation.

class ConductorStateMachine():

    # State descriptions
    # startup: When the system has just been turned on. Light sequences, sound, etc.
    # face_choosing: A face is being chosen from screen 1.
    # face_chosen: A face has been chosen from screen one. Lights or fancy display etc.
    # face_move_1_2: Lights/sounds for a face being moved from screen 1 to screen 2.
    # face_edit: A face is being edited on screen 2, with robot motion and LLM voice and TD visuals. 
    # face_edit_done: The face edit is complete.
    # face_move_2_3: The face is moved from screen 2 to screen 3, where it lives in the Unity world.
    
    states = ['startup', 'face_choosing', 'face_chosen', 'face_move_1_2','face_edit', 
              'face_edit_done', 'face_move_2_3', 'other']
    
    transitions = [

        { 'trigger': 'to_face_choosing', 'source': 'startup', 'dest': 'face_choosing'},

        { 'trigger': 'to_face_chosen', 'source': 'face_choosing', 'dest': 'face_chosen'},

        { 'trigger': 'to_face_move_1_2', 'source': 'face_chosen', 'dest': 'face_move_1_2'},

        { 'trigger': 'to_face_edit', 'source': 'face_move_1_2', 'dest': 'face_edit'},

        { 'trigger': 'to_face_edit_done', 'source': 'face_edit', 'dest': 'face_edit_done'},

        { 'trigger': 'to_face_move_2_3', 'source': 'face_edit_done', 'dest': 'face_move_2_3' },

        { 'trigger': 'to_face_choosing', 'source': 'face_move_2_3', 'dest': 'face_choosing' }

    ]

    def __init__(self):

        # Initialize the state machine

        self.machine = Machine(
            model=self, 
            states=ConductorStateMachine.states, 
            transitions=ConductorStateMachine.transitions, 
            initial='startup'
        )