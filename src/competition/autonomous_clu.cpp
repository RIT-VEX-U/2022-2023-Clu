#include "competition/autonomous_clu.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "automation.h"
#include "vision.h"

const double TURN_SPEED = 0.6;
const double INTAKE_VOLT = 12;
const int  SHOOTING_RPM = 3400;
const int THRESHOLD_RPM = 100;
const double SINGLE_SHOT_TIME = 0.07;
const double SINGLE_SHOT_VOLT = 12;
const int SINGLE_SHOT_RECOVER_DELAY_MS = 800;
const double TRI_SHOT_TIME = 1;
const double TRI_SHOT_VOLT = 9;
const int TRI_SHOT_RECOVER_DELAY_MS = 200;


static void add_single_shot_cmd(CommandController &controller, double timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL, timeout);
    // controller.add(AUTO_AIM, timeout);
    controller.add_delay(250);
    controller.add(SHOOT_DISK);
    controller.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    vexDelay(100);
    controller.add(new StopIntakeCommand(intake));
    controller.add_delay(SINGLE_SHOT_RECOVER_DELAY_MS);
}

static void add_tri_shot_cmd(CommandController &controller, double timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL, timeout);
    controller.add(TRI_SHOT_DISK);
    controller.add_delay(TRI_SHOT_RECOVER_DELAY_MS);
}

CommandController clu_auto_current()
{
    CommandController cmd;

    cmd.add({

        // Init
        new OdomSetPosition(odometry_sys, {.x=128, .y=84, .rot=180}),
        new SpinRPMCommand(flywheel_sys, 3600),
        new FlapDownCommand(),
        
        // Drive to intake 1 (3rd disc)
        START_INTAKE,
        DRIVE_TO_POINT_FAST(108, 84, fwd),
        
        // Turn & Shoot 1 (3 discs)
        TURN_TO_HEADING(155),
        DELAY(500), // Finish indexing
        STOP_INTAKE,
        // AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        new StartIntakeCommand(intake, -12),

        
        // Turn and intake 2 (2 discs)
        TURN_TO_HEADING(227),
        START_INTAKE,

        
        DRIVE_TO_POINT_FAST(84.7,61,fwd),

        // Shoot 2 (2 discs)
        TURN_TO_HEADING(135),
        STOP_INTAKE,
        // AUTO_AIM, // CHANGE TO CHECK_AIM
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        new StartIntakeCommand(intake, -12),

        
        // Turn & intake along barrier (3 discs)
        TURN_TO_HEADING(338),
        DRIVE_TO_POINT_SLOW(90.2,58.3,fwd),
        START_INTAKE,
        
        DRIVE_TO_POINT_SLOW(120,58.7,fwd),
        DRIVE_TO_POINT_SLOW(129,57,fwd),
        
        // Turn and shoot 3 (3 discs)
        TURN_TO_HEADING(150),
        STOP_INTAKE,
        // AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),

        // Turn and go to roller
        TURN_TO_HEADING(90),
        DRIVE_TO_POINT_FAST(123,108,fwd),
        TURN_TO_HEADING(0),
        // TODO roller code
        
        
    });

    return cmd;
}

CommandController clu_auto_rush_current()
{
    CommandController cmd;

    cmd.add({
        // Init
        new OdomSetPosition(odometry_sys, {.x=0, .y=0, .rot=0}),
        new SpinRPMCommand(flywheel_sys, 3600),
        new FlapDownCommand(),

        // Drive to 3 & intake
        START_INTAKE,
        DRIVE_TO_POINT_FAST(0, 0, fwd),
        DELAY(2000), // Index
        STOP_INTAKE,
        new SpinRPMCommand(flywheel_sys, 3600), // Preset for shoot
        DRIVE_TO_POINT_FAST(0, 0, rev),

        // Turn & shoot (3 discs)
        TURN_TO_HEADING(0),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),

        // Intake 2 (3 discs)
        TURN_TO_HEADING(0),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(0, 0, fwd),
        new SpinRPMCommand(flywheel_sys, 3600), // Preset for shoot
        TURN_TO_HEADING(0),
        STOP_INTAKE,

        // Shoot 2 (3 discs)
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),

        // Intake 3 along barrier (3 discs)
        TURN_TO_HEADING(0),
        START_INTAKE,
        DRIVE_TO_POINT_SLOW(0, 0, fwd),
        new SpinRPMCommand(flywheel_sys, 3600), // Preset for shoot
        TURN_TO_HEADING(0),
        STOP_INTAKE,

        // Shoot 3 (3 discs)
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),

        // Intake 4 (preloads, 2 discs)
        TURN_TO_HEADING(0),
        START_INTAKE,
        new SpinRPMCommand(flywheel_sys, 3600), // Preset for shoot
        DRIVE_TO_POINT_FAST(0,0,fwd),
        TURN_TO_HEADING(0),
        STOP_INTAKE,

        // Shoot 4 (2 discs)
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),

        // Drive to Roller
        TURN_TO_HEADING(0),
        DRIVE_TO_POINT_FAST(0, 0, fwd),
        TURN_TO_HEADING(0),


        // Roller
        DRIVE_FORWARD_FAST(0, fwd)->withTimeout(2),
        DRIVE_FORWARD_FAST(0, rev),
        // DRIVE_FORWARD_FAST(0, fwd)->withTimeout(2),
        // DRIVE_FORWARD_FAST(0, rev),

        TURN_TO_HEADING(0),

        // END
    });

    return cmd;
}

CommandController clu_skills_current()
{
    CommandController cmd;
    cmd.add({
        new OdomSetPosition(odometry_sys, {.x = 125.75862, .y = 87.86207, .rot = 90.0}),
        new SpinRPMCommand(flywheel_sys, 3600),

        // Drive to intake 1 (3rd disc)
        START_INTAKE,
        DRIVE_TO_POINT_FAST(125.27586, 126.0, fwd),
        TURN_TO_HEADING(180.0),
        STOP_INTAKE,
        DRIVE_TO_POINT_FAST(93.655174, 126.24138, fwd),

        // Shoot 1 (3 discs)
        TURN_TO_HEADING(180),
        new SpinRPMCommand(flywheel_sys, 3600),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),
        
        // Drive to intake 2 (3 stack)
        TURN_TO_HEADING(295.0),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(109.82759, 90.27586, fwd),
        TURN_TO_HEADING(285.0),
        STOP_INTAKE,

        // Shoot 2 (3 discs)
        new SpinRPMCommand(flywheel_sys, 3600),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),
        
        // Drive to Intake 3 (3 discs)
        TURN_TO_HEADING(80.0),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(114.89655, 115.13793, fwd),
        TURN_TO_HEADING(225.0),
        DRIVE_TO_POINT_FAST(72.17241, 72.41379, fwd),
        TURN_TO_HEADING(135.0),
        STOP_INTAKE,

        // shoot 3 (3 discs)
        new SpinRPMCommand(flywheel_sys, 3600),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),

        // Intake 4 (3 stack)
        TURN_TO_HEADING(75.0),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(82.31035, 109.586205, fwd),
        TURN_TO_HEADING(525.0),
        STOP_INTAKE,

        // Shoot 4 (3 discs)
        new SpinRPMCommand(flywheel_sys, 3600),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),

        // Intake 5
        TURN_TO_HEADING(225.0),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(24.86207, 49.965515, fwd),
        TURN_TO_HEADING(95.0),
        STOP_INTAKE,
        
        // Shoot 5 (3 discs)
        new SpinRPMCommand(flywheel_sys, 3600),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),

        // Intake 6 (3 discs along barrier)
        TURN_TO_HEADING(105.0),
        DRIVE_TO_POINT_FAST(13.034482, 82.55172, fwd),
        TURN_TO_HEADING(15.0),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(42.965515, 86.17242, fwd),
        TURN_TO_HEADING(120.0),
        STOP_INTAKE,

        // Shoot 6 (3 discs center barrier)
        new SpinRPMCommand(flywheel_sys, 3600),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),

        // Intake 7 (3 discs along barrier)
        TURN_TO_HEADING(355.0),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(50.206898, 86.17242, fwd),
        TURN_TO_HEADING(85.0),

        // Shoot 7 (3 discs close to goal)
        new SpinRPMCommand(flywheel_sys, 3600),
        AUTO_AIM,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),

        // Drive to roller 
        TURN_TO_HEADING(180.0),
        DRIVE_TO_POINT_FAST(50.931034, 121.896545, fwd),
         
        // TODO Roller and endgame
        TURN_TO_HEADING(0),
        DRIVE_TO_POINT_FAST(122.86207, 123.344826, fwd),

        new EndgameCommand(endgame_solenoid)
    });

    return cmd;
}


/*
Auto Non-loader side
Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
           ^ 180 degrees
  0-------------------+
  |        |*___| (R) |
  |___           |    |
  |   |          |    |
  |   |          |    |  --> 90 degrees
  |   |          |____|
  |___|  ____         |
  |(B)  |__*_|        |
  +-------------------+
           v 0 degrees

 Human Instructions:
 Align robot to specified place and angle using NON LOADER SIDE AUTO jig
*/
CommandController clu_auto_wv(){

    #define PAUSE return nlsa;
    CommandController nlsa;

    // Initialization
    position_t start_pos = {.x=132, .y=86.5, .rot=90}; 
    nlsa.add(new OdomSetPosition(odometry_sys, start_pos));
    nlsa.add(new SpinRPMCommand(flywheel_sys, SHOOTING_RPM)); // 3400 old
    nlsa.add(new FlapDownCommand());

    // Drive to roller
    nlsa.add(DRIVE_TO_POINT_FAST(130, 107, fwd));
    nlsa.add(TURN_TO_HEADING(0));
    
    // Spin Roller
    nlsa.add(new SpinRollerCommand({.x=135,.y=107,.rot=0}), 10);

    //Drive to center disk
    nlsa.add(TURN_TO_HEADING(252));
    nlsa.add(DRIVE_TO_POINT_FAST(118, 72, fwd));
    nlsa.add(TURN_TO_HEADING(185));
    nlsa.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlsa.add(DRIVE_TO_POINT_SLOW(88, 70, fwd));

    // Shoot!
    nlsa.add(TURN_TO_HEADING(146));
    nlsa.add(new StopIntakeCommand(intake));

    add_single_shot_cmd(nlsa, 1);
    add_single_shot_cmd(nlsa, 1);
    add_single_shot_cmd(nlsa, 1);
    nlsa.add(new StartIntakeCommand(intake, -INTAKE_VOLT)); // Purge unshot disks
    nlsa.add_delay(500);

    // Intake 2.1
    nlsa.add(TURN_TO_HEADING(50));
    nlsa.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlsa.add(DRIVE_TO_POINT_SLOW(97.7,80,fwd));
    // Intake 2.2
    nlsa.add(TURN_TO_HEADING(241));
    nlsa.add(DRIVE_TO_POINT_SLOW(86,59.3,fwd));

    // Back up, aim and shoot!
    nlsa.add(DRIVE_FORWARD_FAST(6,rev));
    nlsa.add(TURN_TO_HEADING(150));
    nlsa.add(new StopIntakeCommand(intake));

    add_single_shot_cmd(nlsa, 1);
    add_single_shot_cmd(nlsa, 1);
    nlsa.add(new StartIntakeCommand(intake, -INTAKE_VOLT)); // Purge unshot disks
    nlsa.add_delay(500);

    nlsa.add(DRIVE_TO_POINT_SLOW(105.2,76.6,rev));

    //Intake 3.1
    nlsa.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlsa.add(DRIVE_TO_POINT_SLOW(109,58,fwd), 2);
    nlsa.add(DRIVE_TO_POINT_FAST(107,70,rev));

    //Intake 3.2
    nlsa.add(DRIVE_TO_POINT_SLOW(103,58,fwd), 2);
    nlsa.add(DRIVE_TO_POINT_FAST(113,76.7,rev));
    //Intake 3.3
    nlsa.add(TURN_TO_HEADING(286));
    nlsa.add(DRIVE_TO_POINT_SLOW(118,61,fwd), 2);
    nlsa.add(DRIVE_TO_POINT_FAST(90,73,rev));
    nlsa.add(new StopIntakeCommand(intake));

    // Shoot!
    nlsa.add(TURN_TO_HEADING(148));
    add_single_shot_cmd(nlsa, 1);
    add_single_shot_cmd(nlsa, 1);
    add_single_shot_cmd(nlsa, 1);
    nlsa.add_delay(500);

    return nlsa;    
}

timer skills_tmr;

/*
Skills Non-loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto

           ^ 180 degrees
  +-------------------+
  |        |*___| (R) |
  |___           |    |
  |   |          |    |
  |   |          |    |  --> 90 degrees
  |   |          |____|
  |___|  ____         |
  |(B)  |__*_|        |
  +-------------------+
           v 0 degrees

 Human Instructions:
 Align robot to specified place and angle using NON LOADER SIDE SKILLS jig
*/
CommandController clu_skills_wv(){

    // Setup
    vision_enabled = true;

    CommandController nlss;
    skills_tmr.reset();

    position_t start_pos = {.x = 132, .y = 56, .rot = 270};
    nlss.add(new OdomSetPosition(odometry_sys, start_pos));
    // nlss.add(new SpinRPMCommand(flywheel_sys, 3400), 1);
    nlss.add(new SpinRPMCommand(flywheel_sys,3000));
    
    // Shoot 1 (2 disks)
    nlss.add(new FlapUpCommand());
    nlss.add(DRIVE_TO_POINT_SLOW(132, 46, fwd));
    nlss.add(TURN_TO_HEADING(256));
    add_tri_shot_cmd(nlss, 3);

    // Roller 1 
    nlss.add(TURN_TO_HEADING(270));
    nlss.add(DRIVE_TO_POINT_FAST(129, 110, rev));
    nlss.add(TURN_TO_HEADING(0));

    nlss.add(new FunctionCommand([](){target_red = true; return true;}));
    // Drive forward and back to roll
    nlss.add(new SpinRollerCommand({.x=134,.y=110,.rot=0}), 7);
    nlss.add(TURN_TO_HEADING(90));

    // Intake Disk 1
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(130, 134, fwd), 3);
    nlss.add(DRIVE_TO_POINT_FAST(127,112,rev));
    nlss.add(TURN_TO_HEADING(128));
    nlss.add(DRIVE_TO_POINT_SLOW(115,127,fwd));
    nlss.add(new StopIntakeCommand(intake));
    nlss.add(TURN_TO_HEADING(80));    

    // Roller 2
    nlss.add(DRIVE_TO_POINT_FAST(115.8, 133, fwd), 4);
    nlss.add(new StopIntakeCommand(intake));
    nlss.add(new SpinRollerCommand({.x=117,.y=137,.rot=90}), 7);
    
    // Intake Disk 3
    // nlss.add(TURN_TO_HEADING(230));
    // nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    // nlss.add(DRIVE_TO_POINT_SLOW(105, 115, fwd));
    // nlss.add(DRIVE_FORWARD_FAST(2, rev));
    // nlss.add(TURN_DEGREES(265));
    
    // Shoot 2 (2 disks)
    
    nlss.add(DRIVE_TO_POINT_FAST(118.4, 94.9, rev));
    nlss.add(new FlapDownCommand());
    nlss.add(new SpinRPMCommand(flywheel_sys,3200));
    nlss.add(new FunctionCommand([](){target_red = false; return true;}));
    nlss.add(TURN_TO_HEADING(278));
    
    add_single_shot_cmd(nlss, 1);
    add_single_shot_cmd(nlss, 1);
    nlss.add(new StartIntakeCommand(intake, -INTAKE_VOLT));
    nlss.add_delay(500);
    
    nlss.add(TURN_TO_HEADING(233));
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(87,55,fwd));
    nlss.add(new StopIntakeCommand(intake));

    nlss.add(new FlapDownCommand());
    nlss.add(new SpinRPMCommand(flywheel_sys,3000));
    nlss.add(TURN_TO_HEADING(317));

    add_single_shot_cmd(nlss, 1);
    add_single_shot_cmd(nlss, 1);
    add_single_shot_cmd(nlss, 1);

    nlss.add(new StartIntakeCommand(intake, -INTAKE_VOLT));

    nlss.add(TURN_TO_HEADING(17));
    nlss.add(DRIVE_TO_POINT_FAST(110,62,fwd));
    nlss.add(TURN_TO_HEADING(270));

    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    // Intake 2 disk 1
    nlss.add(DRIVE_TO_POINT_SLOW(110, 49, fwd), 2);
    nlss.add(DRIVE_TO_POINT_SLOW(104.5, 63.6, rev));

    // Intake 2 disk 2
    nlss.add(DRIVE_TO_POINT_SLOW(102.2, 51.2, fwd),2);
    nlss.add(DRIVE_TO_POINT_SLOW(120.8, 63.9, rev));

    // Intake 2 disk 3
    nlss.add(TURN_TO_HEADING(270));
    nlss.add(DRIVE_TO_POINT_SLOW(122.4,51.9,fwd),2);
    nlss.add(new StopIntakeCommand(intake));
    nlss.add(DRIVE_TO_POINT_SLOW(134, 65, rev));
    
    // Drive to Shooting Pos
    nlss.add(TURN_TO_HEADING(270));
    nlss.add(new FlapUpCommand());
    nlss.add(DRIVE_TO_POINT_FAST(135.6, 44, fwd));
    nlss.add(TURN_TO_HEADING(259));
    add_tri_shot_cmd(nlss, 1.5);
    nlss.add(new StartIntakeCommand(intake, -INTAKE_VOLT));

    nlss.add(DRIVE_TO_POINT_FAST(119.4, 10.2, fwd));
    nlss.add(TURN_TO_HEADING(180));
    nlss.add(DRIVE_TO_POINT_FAST(119.4, 10.2, fwd));

    // Drive to endgame,stop wherever you are at 58 sec
    nlss.add(new FunctionCommand([](){
        double endgame_timeout = 58 - skills_tmr.time(sec);
        CommandController cmd;
        if(endgame_timeout > 0)
            cmd.add(DRIVE_TO_POINT_FAST(32.2, 6.6, fwd), endgame_timeout);
        cmd.run();
        return true;
    }));
    
    nlss.add(new FunctionCommand([](){
        double delta_x = 72 - odometry_sys.get_position().x;
        double delta_y = 144 - odometry_sys.get_position().y;
        double pointat_angle = wrap_angle_deg(rad2deg(atan2(delta_y, delta_x)) + 180);
        CommandController cmd;
        cmd.add(TURN_TO_HEADING(pointat_angle));
        cmd.run();
        return true;
    }));

    nlss.add(new EndgameCommand(endgame_solenoid));
    //-24x
    //132y
    return nlss;

  return nlss;
}