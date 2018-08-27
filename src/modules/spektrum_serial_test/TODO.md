#TODO
##Put in sentinel hover hold value while transitioning between plan elements
That way there is no possibility of creating a wait state where all setpoints
are reset if pose is updated while there are no setpoints

##Fix setpoint holding st held setpoints can still timeout
Or have another option for hold until timeout as an alternative
to hold indefinitely
  This will allow for setpoints to avoid being thrown out if the drone
  is at the setpoint location.

##Allow plan element to be completed if no more setpoints

##Ensure full transform while computing pose
Include roll,pitch angles in determination of body pose

##Decouple pose and motor inputs
Isolate thrust,roll,pitch,yaw based off of x,y,z,q

##Update landing error
Error should be based only on altitude

##Complete simple patterns
Takeoff, land, spin in circle, move about square

##Neural nets
1) Motor inputs
2) Trajectory updates
  - i.e. given a trajectory error & environment
    derived from cellular automata, update trajectory
3) Trajectory generation
  - i.e. given a goal, environment, & other metrics
    (battery life? urgency? stability estimate?)
    generate trajectory

##Networking w/IBSS wifi


##Add routing table to message manager
###Update message format to include source and destination
###Add message forwarding

##Driver for packing packets
### Include elka_packet.msg and update accordingly
Should figure out whether to write new packet or
append to existing packet
