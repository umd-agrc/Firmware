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

uint32 TRAJ_DIR_FWD = 1 # 0x1
uint32 TRAJ_DIR_BACK = 2 # 0x1 << 1
uint32 TRAJ_DIR_LEFT = 4 # 0x1 << 2
uint32 TRAJ_DIR_RIGHT = 8 # 0x1 << 3
uint32 TRAJ_DIR_UP = 16 # 0x1 << 4
uint32 TRAJ_DIR_DOWN = 32 # 0x1 << 5

uint8 trajectory_direction
float32 trajectory_radius
float32 dpsi

float32[3] init_pos
float32[3] final_pos
float32[3] init_vel
float32[3] final_vel
float32[3] init_acc
float32[3] final_acc
