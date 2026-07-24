std_msgs/Header header

uint8 UNINITIALIZED=0
uint8 WAITING_FOR_MAP=1
uint8 WAITING_FOR_LIO=2
uint8 INITIALIZING=3
uint8 TRACKING=4
uint8 RELOCALIZING=5
uint8 LOST=6
uint8 ERROR=7

uint8 state
string state_text

bool initialized
bool tracking
bool icp_converged

float64 icp_fitness_score
float64 translation_error
float64 rotation_error_deg
