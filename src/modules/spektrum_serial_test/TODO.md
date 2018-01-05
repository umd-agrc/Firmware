#TODO
##Send {X,Y,Z,VX,VY,VZ,Yaw,VYaw} to ELKA

##Add trajectory to BasicNavigator

##Complete simple patterns
Takeoff, land, spin in circle, move about square

##Neural net gain update predictions

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
