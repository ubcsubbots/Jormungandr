Purpose:

States, or subroutines, describe the tactical goal of the robot at any given moment: locating the gate, passing the gate, pushing the dice, etc.

States are responsible for interfacing with their respective sensor data nodes and retaining some knowledge of other states, as it tells the overarching WorldStateNode which state to transition to next. i.e. It must have knowledge of both communicating and accessible nodes.

The WorldStateNode is a finite state machine. However, it doesn't know how the routines are making their decisions. It only knows what state to call. 
