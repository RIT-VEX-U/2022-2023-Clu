#include "competition/autonomous_clu.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "automation.h"
#include "vision.h"

const double TURN_SPEED = 0.6;
const double INTAKE_VOLT = 12;
const int  SHOOTING_RPM = 3400;
const int THRESHOLD_RPM = 100;
const double SINGLE_SHOT_TIME = 0.065;
const double SINGLE_SHOT_VOLT = 12;
const int SINGLE_SHOT_RECOVER_DELAY_MS = 400;
const double TRI_SHOT_TIME = 1;
const double TRI_SHOT_VOLT = 9;
const int TRI_SHOT_RECOVER_DELAY_MS = 200;
const bool AIM_ODOM_FALLBACK = true;
const int AIM_CENTER = 155;
const int AIM_FALLBACK_DEGREES = 10;
const int INDEX_DELAY = 500;


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

CommandController clu_auto_current_pt1()
{
    CommandController cmd;

    cmd.add({

        // Init
        new OdomSetPosition(odometry_sys, {.x=128, .y=84, .rot=180}),
        new SpinRPMCommand(flywheel_sys, 3250),
        new FlapDownCommand(),
        
        // Drive to intake 1 (3rd disc)
        START_INTAKE,
        DRIVE_TO_POINT_FAST(108, 83, fwd),
        
        // Turn & Shoot 1 (3 discs)
        TURN_TO_HEADING(155),
        DELAY(500), // Finish indexing
        STOP_INTAKE,
        
        (new VisionAimCommand(true,157,20))->withTimeout(2),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        new StartIntakeCommand(intake, -12),

        
        // Turn and intake 2 (2 discs)
        new SpinRPMCommand(flywheel_sys, 3100),
        TURN_TO_HEADING(227),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(85.4,60.4,fwd),

        // Shoot 2 (2 discs)
        TURN_TO_HEADING(138),
        STOP_INTAKE,
        (new VisionAimCommand(true,156,20))->withTimeout(2),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        new StartIntakeCommand(intake, -12),

    });

    return cmd;
}

CommandController clu_auto_current_pt2()
{
    CommandController cmd;
    cmd.add({
        
        // Turn & intake along barrier (3 discs)
        new SpinRPMCommand(flywheel_sys, 3200),
        TURN_TO_HEADING(356),
        
        DRIVE_TO_POINT_SLOW(99.6,58,fwd),
        START_INTAKE,
        
        DRIVE_TO_POINT_SLOW(125,59.7,fwd),
        DRIVE_TO_POINT_SLOW(131,58,fwd),
        
        // Turn and shoot 3 (3 discs)
        TURN_TO_HEADING(155),
        
        DRIVE_TO_POINT_FAST(98.1,81.4,fwd),
        TURN_TO_HEADING(155),
        STOP_INTAKE,
        (new VisionAimCommand(true,148,20))->withTimeout(2),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),

        
        // Turn and go to roller
        TURN_TO_HEADING(54),
        
        DRIVE_TO_POINT_FAST(126,111,fwd),
        TURN_TO_HEADING(0),
        // TODO roller code
        new FunctionCommand([](){ roller_sensor.setLightPower(100, pct); return true; }),
        new SpinRollerCommand(),
        new FunctionCommand([](){ roller_sensor.setLightPower(0, pct); return true; })
        
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
        new FunctionCommand([](){
            target_red = true;
            vision_enabled = true;
            return true;
        }),
        new OdomSetPosition(odometry_sys, {.x = 132, .y = 90, .rot = 90.0}),
        new FlapDownCommand(),
        
        // Drive to intake 1 (3rd disc)
        new SpinRPMCommand(flywheel_sys, 3000),
        DRIVE_TO_POINT_FAST(130, 125, fwd),
        START_INTAKE,
        DRIVE_TO_POINT_SLOW(130, 133, fwd),
        TURN_TO_HEADING(180.0),
        DRIVE_TO_POINT_FAST(95, 133, fwd),
        STOP_INTAKE,
        
        // Shoot 1 (3 discs)
        TURN_TO_HEADING(182.6),
        (new VisionAimCommand(true, 144, 45))->withTimeout(3),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),
        new StartIntakeCommand(intake, -12),
        
        // Drive to intake 2 (3 stack)
        new SpinRPMCommand(flywheel_sys, 3250),
        TURN_TO_HEADING(290.8),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(102.5, 116.7, fwd),
        DRIVE_TO_POINT_SLOW(111, 97.6, fwd),
        
        // Shoot 2 (3 discs)
        TURN_TO_HEADING(160),
        DELAY(INDEX_DELAY), //Finish indexing
        STOP_INTAKE,
        (new VisionAimCommand(true, 148, 45))->withTimeout(3),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS + 300),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS + 300),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS + 300),
        SHOOT_DISK,
        DELAY(500),
        new StartIntakeCommand(intake, -12),
        
        // Drive to Intake 3 (3 discs)
        new SpinRPMCommand(flywheel_sys, 2900),
        TURN_TO_HEADING(97),
        START_INTAKE,
        DRIVE_TO_POINT_SLOW(114.7, 121.3, fwd),
        TURN_TO_HEADING(221),
        DRIVE_TO_POINT_FAST(100.1, 107.9, fwd),
        DRIVE_TO_POINT_FAST(80, 87.3, fwd),
        
        // shoot 3 (3 discs)
        TURN_TO_HEADING(149),
        DELAY(INDEX_DELAY), // Finish indexing
        STOP_INTAKE,
        (new VisionAimCommand(true, 151, 45))->withTimeout(3),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),
        new StartIntakeCommand(intake, -12),

        // Intake 4 (3 stack)
        new SpinRPMCommand(flywheel_sys, 2800),
        TURN_TO_HEADING(90),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(77, 104.5, fwd),
        DRIVE_TO_POINT_SLOW(76.3, 125, fwd),
        
        // Shoot 4 (3 discs)
        TURN_TO_HEADING(175),
        DELAY(INDEX_DELAY), //Finish indexing
        STOP_INTAKE,
        (new VisionAimCommand(true, 145, 45))->withTimeout(3),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),
        new StartIntakeCommand(intake, -12),

        // Intake 5
        new SpinRPMCommand(flywheel_sys, 2800),
        TURN_TO_HEADING(234),
        START_INTAKE,
        DRIVE_TO_POINT_FAST(53, 91, fwd),
        TURN_TO_HEADING(218),
        DRIVE_TO_POINT_FAST(23.4, 64.0, fwd),
        
        // Shoot 5 (3 discs)
        TURN_TO_HEADING(103),
        DELAY(INDEX_DELAY), // Finish Indexing
        STOP_INTAKE,
        (new VisionAimCommand(true, 144, 45))->withTimeout(3),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),
        new StartIntakeCommand(intake, -12),

        // Intake 6 (3 discs along barrier)
        new SpinRPMCommand(flywheel_sys, 2800),
        DRIVE_TO_POINT_FAST(7.0, 90.6, fwd),
        TURN_TO_HEADING(33),
        START_INTAKE,
        DRIVE_TO_POINT_SLOW(19, 93, fwd),
        DRIVE_TO_POINT_SLOW(52, 97, fwd),

        // Shoot 6 (3 discs center barrier)
        TURN_TO_HEADING(136),
        DELAY(INDEX_DELAY), // Finish Indexing
        STOP_INTAKE,
        (new VisionAimCommand(true, 148, 45))->withTimeout(3),
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(SINGLE_SHOT_RECOVER_DELAY_MS),
        SHOOT_DISK,
        DELAY(500),

        // Drive to Roller
        TURN_TO_HEADING(19.6),
        DRIVE_TO_POINT_FAST(117, 117, fwd),
        TURN_TO_HEADING(2),
        (new FunctionCommand([](){ roller_sensor.setLightPower(100, pct); return true; })),
        (new SpinRollerCommand())->withTimeout(5),
        (new FunctionCommand([](){ roller_sensor.setLightPower(0, pct); return true; })),
        DRIVE_TO_POINT_FAST(109, 126, rev),
        TURN_TO_HEADING(45),
        new EndgameCommand(endgame_solenoid),
        // END
        
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