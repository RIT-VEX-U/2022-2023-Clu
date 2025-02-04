#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

// Drive
motor left_front(PORT20, vex::gearSetting::ratio18_1, true), left_mid(PORT8, vex::gearSetting::ratio18_1, true), left_rear(PORT7, vex::gearSetting::ratio18_1, true);
motor right_front(PORT11, vex::gearSetting::ratio18_1), right_mid(PORT5, vex::gearSetting::ratio18_1), right_rear(PORT1, vex::gearSetting::ratio18_1);

motor_group left_motors(left_front, left_mid, left_rear);
motor_group right_motors(right_front, right_mid, right_rear);

// Manipulation 
motor flywheel(PORT14);
motor intake(PORT13);
motor roller(PORT4);

motor_group flywheel_motors(flywheel);

std::map<std::string, motor &> motor_names{
    {"left front", left_front},
    {"left mid", left_mid},
    {"left rear", left_rear},

    {"right front", right_front},
    {"right mid", right_mid},
    {"right rear", right_rear},

    {"flywheel", flywheel},
    {"intake", intake},

};


// Other Outputs
digital_out endgame_solenoid(Brain.ThreeWirePort.H);
digital_out flapup_solenoid(Brain.ThreeWirePort.G);

// ======== INPUTS ========
CustomEncoder left_enc(Brain.ThreeWirePort.A, 2048);
CustomEncoder right_enc(Brain.ThreeWirePort.C, 2048);

inertial imu(PORT15);
vex::analog_in mode_switch(Brain.ThreeWirePort.F);
optical roller_sensor(PORT4);

// ======== UTILS ========

// Drive Tuning
PID::pid_config_t drive_pid_cfg = {
    .p = .015,
    .i = 0, 
    .d = .008,
    .deadband = 2,
    .on_target_time = 0.2
};

FeedForward::ff_config_t drive_ff_cfg = {
    .kS = 0.03,
    .kV =.0115,    //.0138, 
    .kA = 0.001
};

MotionController::m_profile_cfg_t drive_fast_mprofile_cfg = {
    .max_v = 60,// MAX = 60,
    .accel = 100, // MAX = 200
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg
};

MotionController::m_profile_cfg_t drive_slow_mprofile_cfg = {
    .max_v = 20,
    .accel = 100,
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg    
};


// Turn Tuning
PID::pid_config_t turn_pid_cfg = {
    .p = .015,
    .i = 0, 
    .d = .001,
    .deadband = 5,
    .on_target_time = .2
};

FeedForward::ff_config_t turn_ff_cfg = {
    .kS = 0.11,
    .kV = 0.001,
    .kA = .00017
};

// MotionController::m_profile_cfg_t turn_fast_mprofile_cfg = {
//     .max_v = 600, //700,
//     .accel = 1000, //1400,
//     .pid_cfg = turn_pid_cfg,
//     .ff_cfg = turn_ff_cfg
// };

// MotionController::m_profile_cfg_t turn_slow_mprofile_cfg = {
//     .max_v = 0,
//     .accel = 0,
//     .pid_cfg = turn_pid_cfg,
//     .ff_cfg = turn_ff_cfg
// };

MotionController drive_fast_mprofile(drive_fast_mprofile_cfg), drive_slow_mprofile(drive_slow_mprofile_cfg);
// MotionController turn_fast_mprofile(turn_fast_mprofile_cfg), turn_slow_mprofile(turn_slow_mprofile_cfg);

robot_specs_t config = {
    .robot_radius = 10,
    .odom_wheel_diam = 2.731, //6.374, // OLD - MOTOR ENCODERS
    .odom_gear_ratio = 1, // .44    16:12
    .dist_between_wheels = 10.469, //10.24, // OLD - MOTOR ENCODERS

    .drive_correction_cutoff = 5,

    .drive_feedback = &drive_fast_mprofile,
    .turn_feedback = new PIDFF(turn_pid_cfg, turn_ff_cfg),
    .correction_pid = {
        .p = .02, //.012
        .i = 0,
        .d = 0.0012
    }
};

// Flywheel Tuning
FeedForward::ff_config_t flywheel_ff_cfg = {
  .kV =  0.000281
};

PID::pid_config_t flywheel_pid_cfg = {
    .p = .002,
    // .d = 0.000015,
};

// ======== SUBSYSTEMS ========


OdometryTank odometry_sys(left_enc, right_enc, config, &imu);
// OdometryTank odometry_sys(left_enc, right_enc, config);
// OdometryTank odometry_sys(left_motors, right_motors, config, &imu);

TankDrive drive_sys(left_motors, right_motors, config, &odometry_sys);

Flywheel flywheel_sys(flywheel_motors, flywheel_pid_cfg, flywheel_ff_cfg, 18);
vex::timer oneshot_tmr;

AutoChooser autochooser(Brain);


std::string AutoLoaderSideDisplayName = "Auto Loader Side";
std::string AutoNonLoaderSideDisplayName = "Auto Non Loader Side";
std::string SkillsLoaderSideDisplayName = "Skills Loader Side";
std::string SkillsNonLoaderSideDisplayName = "Skills Non Loader Side";

bool target_red = true;
bool vision_enabled = false;
int num_roller_fallback = 2;

RobotMode curr_mode = INIT;

#define MODE_AUTOSKILLS_PCT 74
#define MODE_COMP_PCT 25
#define MODE_TEST1_PCT 10

void select_mode()
{
    mode_switch.value(pct);
    vexDelay(500);
    int ang = mode_switch.value(pct);

    if(ang > MODE_AUTOSKILLS_PCT)
        curr_mode = AUTOSKILLS;
    else if (ang > MODE_COMP_PCT)
        curr_mode = COMP;
    else if (ang > MODE_TEST1_PCT)
        curr_mode = TEST1;
    else
        curr_mode = TEST2;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
    endgame_solenoid.set(false); //TODO figure out if false or true shoots 
    
    // Select the mode on setup, and whenever the pot changes
    // select_mode();
    // mode_switch.changed(select_mode);

    imu.calibrate();
}