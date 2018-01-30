#TODO
##Ensure full transform while computing pose
Include roll,pitch angles in determination of body pose

##Decouple pose and motor inputs
Isolate thrust,roll,pitch,yaw based off of x,y,z,q

##Update landing error
Error should be based only on altitude

##Complete simple patterns
Takeoff, land, spin in circle, move about square

##Neural net gain update predictions
Receive from elka in main loop
Transmit to NN in main loop
Receive from NN in main loop
Fitness function in BasicController
Transmit to elka in BasicController
Initial error is difference from ELKA input

Eventually train angular gains and linear motor inputs
Angular inputs must be gains b/c inner loop update rate is much
higher than outer loop position estimate rate

##Neural net position estimates
Heuristic equation to determine when to trust vision pose estimates
and when to trust neural net
Train neural net with
  {vicon data} vs {sensor data,vision pose estimates}

##Add routing table to message manager
###Update message format to include source and destination
###Add message forwarding

##Driver for packing packets
### Include elka_packet.msg and update accordingly
Should figure out whether to write new packet or
append to existing packet

##Determine when we are sufficiently close to a setpoint 

##Send a series of setpoints
