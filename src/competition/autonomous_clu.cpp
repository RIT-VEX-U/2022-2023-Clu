#include "competition/autonomous_clu.h"
#include "robot-config.h"
#include "automation.h"
#include "vision.h"

#define TURN_SPEED 0.6
#define INTAKE_VOLT 12
#define SHOOTING_RPM 3450
#define THRESHOLD_RPM 50
#define SINGLE_SHOT_TIME 0.1
#define SINGLE_SHOT_VOLT 9
#define SINGLE_SHOT_RECOVER_DELAY_MS 200
#define TRI_SHOT_TIME 1
#define TRI_SHOT_VOLT 9
#define TRI_SHOT_RECOVER_DELAY_MS 200

// drive commands
#define DRIVE_TO_POINT_FAST(x,y,dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_SLOW(x,y,dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, directionType::dir))
#define DRIVE_FORWARD_FAST(in, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, in, directionType::dir))
#define DRIVE_FORWARD_SLOW(in, dir) (new DriveForwardCommand(drive_sys, drive_slow_mprofile, in, directionType::dir))

// turn commands
#define TURN_TO_HEADING(dir) (new TurnToHeadingCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_DEGREES(dir) (new TurnDegreesCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_TO_POINT(x, y) (new TurnToPointCommand(drive_sys, odometry_sys, *config.turn_feedback, {x, y}))

// shooting commands
#define AUTO_AIM (new VisionAimCommand())
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, THRESHOLD_RPM))
#define SHOOT_DISK (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define TRI_SHOT_DISK (new ShootCommand(intake, TRI_SHOT_TIME, TRI_SHOT_VOLT))

static void add_single_shot_cmd(CommandController &controller, double timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL, timeout);
    controller.add(AUTO_AIM, timeout);
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

/*
Auto Non-loader side
JOEBOT
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
CommandController auto_non_loader_side(){

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
    nlsa.add(TURN_TO_HEADING(150));
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
    nlsa.add(TURN_TO_HEADING(154));
    nlsa.add(new StopIntakeCommand(intake));

    add_single_shot_cmd(nlsa, 1);
    add_single_shot_cmd(nlsa, 1);
    nlsa.add(new StartIntakeCommand(intake, -INTAKE_VOLT)); // Purge unshot disks
    nlsa.add_delay(500);

    nlsa.add(DRIVE_TO_POINT_SLOW(105.2,76.6,rev));

    //Intake 3.1
    nlsa.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlsa.add(DRIVE_TO_POINT_SLOW(107.2,59,fwd));
    nlsa.add(DRIVE_TO_POINT_FAST(105.7,70,rev));

    //Intake 3.2
    nlsa.add(DRIVE_TO_POINT_SLOW(101.5,59,fwd));
    nlsa.add(DRIVE_TO_POINT_FAST(111.3,76.7,rev));
    //Intake 3.3
    nlsa.add(TURN_TO_HEADING(286));
    nlsa.add(DRIVE_TO_POINT_SLOW(116,62,fwd));
    nlsa.add(DRIVE_TO_POINT_FAST(88,75,rev));
    nlsa.add(new StopIntakeCommand(intake));

    // Shoot!
    nlsa.add(TURN_TO_HEADING(142));
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
CommandController prog_skills_non_loader_side(){

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
    nlss.add(DRIVE_TO_POINT_SLOW(130, 134, fwd));
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

    nlss.add(new FlapUpCommand());
    nlss.add(new SpinRPMCommand(flywheel_sys,3050));
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
    nlss.add(DRIVE_TO_POINT_SLOW(110, 49, fwd));
    nlss.add(DRIVE_TO_POINT_SLOW(104.5, 63.6, rev));

    // Intake 2 disk 2
    nlss.add(DRIVE_TO_POINT_SLOW(102.2, 51.2, fwd));
    nlss.add(DRIVE_TO_POINT_SLOW(120.8, 63.9, rev));

    // Intake 2 disk 3
    nlss.add(TURN_TO_HEADING(270));
    nlss.add(DRIVE_TO_POINT_SLOW(122.4,51.9,fwd));
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
    //-24x
    //132y
    return nlss;

  return nlss;
}