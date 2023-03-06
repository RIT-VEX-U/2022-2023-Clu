/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature RED_GOAL = vex::vision::signature (1, 8383, 10625, 9504, -1227, 1, -613, 5.3, 0);
vex::vision::signature BLUE_GOAL = vex::vision::signature (2, -2499, -927, -1713, 7389, 11129, 9259, 3, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision cam = vex::vision (vex::PORT2, 150, RED_GOAL, BLUE_GOAL, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/