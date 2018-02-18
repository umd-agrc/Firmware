#TODO
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
