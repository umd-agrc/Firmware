enum TrajectoryDirection { 
  Fwd = 0x1,
  Back = 0x01 << 1,
  Left = 0x01 << 2,
  Right = 0x01 << 3,
  Up = 0x01 << 4,
  Down = 0x01 << 5
};

uint8 TYPE_CHECK = 0
uint8 TYPE_CALIBRATE = 1
uint8 TYPE_TRAJECTORY = 2

uint8 type
uint64 dt

uint8 TRAJ_TYPE_TAKEOFF = 0
uint8 TRAJ_TYPE_LAND = 1
uint8 TRAJ_TYPE_HOVER = 2
uint8 TRAJ_TYPE_LINE = 3
uint8 TRAJ_TYPE_CIRCLE = 4
uint8 TRAJ_TYPE_SQUARE = 5
uint8 TRAJ_TYPE_SPIRAL = 6
uint8 TRAJ_TYPE_SPIN = 7

uint8 trajectory_type

uin32 TRAJ_DIR_FWD = 1 # 0x1
uin32 TRAJ_DIR_BACK = 2 # 0x1 << 1
uin32 TRAJ_DIR_LEFT = 4 # 0x1 << 2
uin32 TRAJ_DIR_RIGHT = 8 # 0x1 << 3
uin32 TRAJ_DIR_UP = 16 # 0x1 << 4
uin32 TRAJ_DIR_DOWN = 32 # 0x1 << 5

uint8 trajectory_direction
float32 trajectory_radius
float32 dpsi

float32 init_pos[3]
float32 final_pos[3]
float32 init_vel[3]
float32 final_vel[3]
float32 init_acc[3]
float32 final_acc[3]
