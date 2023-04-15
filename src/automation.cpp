#include <cstdio>
#include "automation.h"
#include "robot-config.h"
#include "vision.h"

// Pushes the firing flap to the up position for close in shots
void flap_up()
{
  flapup_solenoid.set(false);
}

// Pushes the firing flap to the down position for far away shots
void flap_down()
{
  flapup_solenoid.set(true);
}
/**
 * Construct a FlapUpCommand
 * when run it flaps the flap up
 */
FlapUpCommand::FlapUpCommand() {}
bool FlapUpCommand::run()
{
  flap_up();
  return true;
}
/**
 * Construct a FlapDownCommand
 * when run it flaps the flap down
 */
FlapDownCommand::FlapDownCommand() {}
bool FlapDownCommand::run()
{
  flap_down();
  return true;
}

/**
 * Construct a SpinRollerCommand
 * @param roller_motor The motor that will spin the roller
 */
SpinRollerCommand::SpinRollerCommand(position_t align_pos): align_pos(align_pos), roller_count(0) {}

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */

bool SpinRollerCommand::run()
{

  vexDelay(100);
  Pepsi cur_roller = scan_roller();
  printf("%s\n", cur_roller==RED?"red":cur_roller==BLUE?"blue":"neutral");
  if(vision_enabled && (cur_roller == RED && target_red) || cur_roller == BLUE && !target_red)
  {
    drive_sys.stop();
    return true;
  }

  CommandController cmd;
  cmd.add({
          (new DriveForwardCommand(drive_sys, drive_fast_mprofile, 12, directionType::fwd))->withTimeout(0.5),
           (new DelayCommand(100)),
           (new OdomSetPosition(odometry_sys, align_pos)),
           (new DriveForwardCommand(drive_sys, drive_fast_mprofile, 6, directionType::rev))
  });

  cmd.run();

  if(!vision_enabled && ++roller_count >= num_roller_fallback)
    return true;

  return false;
}

void SpinRollerCommand::on_timeout()
{
  drive_sys.stop();
}

/**
 * Construct a ShootCommand
 * @param firing_motor The motor that will spin the disk into the flywheel
 */
ShootCommand::ShootCommand(vex::motor &firing_motor, double seconds_to_shoot, double volt) : firing_motor(firing_motor), seconds_to_shoot(seconds_to_shoot), volt(volt) {}

/**
 * Run the intake motor backward to move the disk into the flywheel
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool ShootCommand::run()
{
  if (!func_initialized)
  {
    tmr.reset();
    func_initialized = true;
  }

  if (tmr.time(vex::seconds) > seconds_to_shoot)
  {
    func_initialized = false;
    firing_motor.stop();
    return true;
  }
  printf("Shooting at %f RPM\n", flywheel_sys.getRPM());
  firing_motor.spin(vex::fwd, volt, vex::volt); // TODO figure out if this needs to be negated to slap it into the flywheel
  return false;
}

/**
 * Construct a StartIntakeCommand
 * @param intaking_motor The motor that will pull the disk into the robot
 * @param intaking_voltage The voltage at which to run the intake motor
 */
StartIntakeCommand::StartIntakeCommand(vex::motor &intaking_motor, double intaking_voltage) : intaking_motor(intaking_motor), intaking_voltage(intaking_voltage) {}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise

*/
bool StartIntakeCommand::run()
{
  intaking_motor.spin(vex::reverse, intaking_voltage, vex::volt);
  return true;
}

SpinRawCommand::SpinRawCommand(vex::motor &flywheel_motor, double voltage) : flywheel_motor(flywheel_motor), voltage(voltage) {}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise

*/
bool SpinRawCommand::run()
{
  flywheel_motor.spin(vex::fwd, voltage, vex::volt);
  return true;
}

/**
 * Construct a StartIntakeCommand
 * @param intaking_motor The motor that will be stopped
 */
StopIntakeCommand::StopIntakeCommand(vex::motor &intaking_motor) : intaking_motor(intaking_motor) {}

/**
 * Run the StopIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool StopIntakeCommand::run()
{
  intaking_motor.stop();
  return true;
}

EndgameCommand::EndgameCommand(vex::digital_out &solenoid) : solenoid(solenoid) {}
bool EndgameCommand::run()
{
  solenoid.set(true);
  return true;
}
PrintOdomCommand::PrintOdomCommand(OdometryTank &odom) : odom(odom) {}

bool PrintOdomCommand::run()
{
  position_t pos = odom.get_position();
  printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
  return true;
}

PrintOdomContinousCommand::PrintOdomContinousCommand(OdometryTank &odom) : odom(odom) {}
bool PrintOdomContinousCommand::run()
{
  position_t pos = odom.get_position();
  printf("(%.2f, %.2f), %.2f\n", pos.x, pos.y, pos.rot);
  return false;
}
PID::pid_config_t vis_pid_cfg = {
    .p = .003,
    // .d = .0001,
    .deadband = 5,
    .on_target_time = .2};

FeedForward::ff_config_t vis_ff_cfg = {
    .kS = 0.07};

#define MIN_AREA 500
#define MIN_SMALL_AREA 125
#define MAX_SMALL_AREA 2205

#define MIN_LARGE_AREA 114
#define MAX_LARGE_AREA 3158
#define MAX_SPEED 0.5

VisionAimCommand::VisionAimCommand(bool odometry_fallback, int vision_center, int fallback_degrees)
    : pidff(vis_pid_cfg, vis_ff_cfg), odometry_fallback(odometry_fallback), first_run(true), fallback_triggered(false), vision_center(vision_center), fallback_degrees(fallback_degrees)
{
}

/**
 * Run the VisionAimCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool VisionAimCommand::run()
{
  if (first_run)
  {
    stored_pos = odometry_sys.get_position();
    drive_sys.reset_auto();
    first_run = false;
  }

  if (odometry_fallback &&
      (fallback_triggered || fabs(OdometryBase::smallest_angle(stored_pos.rot, odometry_sys.get_position().rot)) > fallback_degrees))
  {
    fallback_triggered = true;
    if (drive_sys.turn_to_heading(stored_pos.rot, 0.6))
      return true;
    else
      return false;
  }

  // If the camera isn't installed, move on to the next command
  if (!cam.installed())
    return true;
  // If we have disabled vision on the screen, move on to the next command
  if (!vision_enabled)
    return true;

  // Take a snapshot with each color selected,
  // and store the largest found object for each in a vector
  //vision::object red_obj, blue_obj;

  int x_val = 0;
  // Get largest red blob
  
  // int red_count = cam.objectCount;
  // if (red_count > 0){
    
  // }
  if(target_red){
    cam.takeSnapshot(RED_GOAL);
  } else {
    cam.takeSnapshot(BLUE_GOAL);
  }

  vector<vex::vision::object> small_blobs;
  vector<vex::vision::object> large_blobs;


  printf("Object Count %ld\n", cam.objectCount);
  // for(int i = 0; i < cam.objectCount; i++){
  //   double area = cam.objects[i].width * cam.objects[i].height;
  //   if(area >= MIN_SMALL_AREA && area <= MAX_SMALL_AREA){
  //     small_blobs.push_back(cam.objects[i]);
  //   }else if(area >= MIN_LARGE_AREA && area <= MAX_LARGE_AREA){
  //     large_blobs.push_back(cam.objects[i]);
  //   }
  // }

  // int outer_loop_size;
  // int inner_loop_size;
  
  // if(small_blobs.size() > large_blobs.size()){
  //   outer_loop_size = small_blobs.size();
  //   inner_loop_size = large_blobs.size();
  // } else {
  //   outer_loop_size = large_blobs.size();
  //   inner_loop_size = small_blobs.size();
  // }
  
  // for(int i = 0; i < outer_loop_size; i++){
  //   for(int j = 0; j < inner_loop_size; j++){
  //     if(fabs(small_blobs[i].centerX - large_blobs[j].centerX) <= 5){
  //       x_val = small_blobs[i].centerX;
  //     }
  //   }
  // }
  for(int i = 0; i < cam.objectCount; i++){
    double blob_1_area = cam.objects[i].width * cam.objects[i].height;
    if(blob_1_area < MIN_LARGE_AREA || blob_1_area > MAX_LARGE_AREA){
      continue;
    }
      for(int j = i+1; j < cam.objectCount; j++){
        double blob_2_area = cam.objects[j].width * cam.objects[j].height;
        if(fabs(cam.objects[i].centerX - cam.objects[j].centerX) < 5 && blob_2_area >= MIN_LARGE_AREA && blob_2_area <= MAX_LARGE_AREA){
          x_val = cam.objects[i].centerX;
        }
      }
    }



  // // Compare the areas of the largest
  // double red_area = red_obj.width * red_obj.height;
  // double blue_area = blue_obj.width * blue_obj.height;
  

  // if (red_area > blue_area && red_area > MIN_AREA && target_red)
  // {
  //   x_val = red_obj.centerX;
  // }
  // else if (blue_area > red_area && blue_area > MIN_AREA && !target_red)
  // {
  //   x_val = blue_obj.centerX;
  // }

  printf("CenterX: %d\n", x_val);

  if (x_val != 0)
  {

    // Update the PID loop & drive the robot
    pidff.set_target(vision_center);
    pidff.set_limits(-MAX_SPEED, MAX_SPEED);
    double out = pidff.update(x_val);

    // Currently set up for upside-down camera. Flip signs if going backwards.
    drive_sys.drive_tank(out, -out);

    if (pidff.is_on_target())
    {
      drive_sys.stop();
      return true;
    }
  }
  else
  {
    drive_sys.stop();
    printf("Nothing Found\n");
  }

  return false;
}

void VisionAimCommand::on_timeout()
{
  drive_sys.stop();
}

/**
 * Constuct a TurnToPointCommand
 * @param odom Refrence to the OdometryTank object
 * @param drive_sys Reference to the TankDrive system
 * @param point The point we want to turn towards
 */
TurnToPointCommand::TurnToPointCommand(TankDrive &drive_sys, OdometryTank &odom, Feedback &turn_feedback, Vector2D::point_t point) : drive_sys(drive_sys), odom(odom), feedback(turn_feedback), point(point) {}

/**
 * Run the TurnToPointCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnToPointCommand::run()
{
  // get differences in x and y to calculate angle relative to x axis
  double delta_x = point.x - odom.get_position().x;
  double delta_y = point.y - odom.get_position().y;

  // get the angle
  double heading_deg = rad2deg(atan2(delta_y, delta_x));

  return drive_sys.turn_to_heading(heading_deg, feedback);
}

/**
 * Constuct a FunctionCommand
 * @param func the function to run
 */
FunctionCommand::FunctionCommand(std::function<bool(void)> func) : func(func) {}

/**
 * Run the TurnToPointCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool FunctionCommand::run()
{
  return func();
}
/**
 * Align with a wall at a certain x, y and heading.
 * If a value is not known at compile time, set it to NO_CHANGE and it will be filled in when the command is run
 * @param drive_sys how to drive into the wall
 * @param odom how to know where we hit a wall
 * @param x where we hit the wall in the x dimension (NO_CHANGE if this is the unknown quantity)
 * @param y where we hit the wall in the y dimension (NO_CHANGE if this is the unknown quantity)
 * @param heading the angle we want to hit the wall at (should never be no change)
 * @param drive_power how fast we want it hit the wall
 */
WallAlignCommand::WallAlignCommand(TankDrive &drive_sys, OdometryTank &odom, double x, double y, double heading, double drive_power, double time) : drive_sys(drive_sys), odom(odom), x(x), y(y), heading(heading), time(time), func_initialized(false) {}

/**
 * reset the position to that which is specified
 */
bool WallAlignCommand::run()
{
  // Start cutoff timer
  if (!func_initialized)
  {
    tmr.reset();
    func_initialized = true;
  }

  // If we're not cutoff, drive
  if (tmr.time(seconds) < time)
  {
    drive_sys.drive_tank(drive_power, drive_power);
    return false;
  }
  // otherwise stop and reset the position
  drive_sys.stop();

  position_t old_pos = odom.get_position();
  position_t newpos = {.x = x, .y = y, .rot = heading};
  if (x == NO_CHANGE)
  {
    newpos.x = old_pos.x;
  }
  if (x == NO_CHANGE)
  {
    newpos.y = old_pos.y;
  }
  odom.set_position(newpos);
  return true;
}

#define ROLLER_AREA_CUTOFF 1500

Pepsi scan_roller()
{
  if (!cam.installed())
  {
    printf("Cam Disconnected!\n");
    return NEUTRAL;
  }

  // SCAN FOR RED
  cam.takeSnapshot(RED_GOAL);
  vex::vision::object red_obj = cam.largestObject;
  int red_area = red_obj.width * red_obj.height;
  int red_y = red_obj.centerY;

  if (cam.objectCount < 1 || red_area < ROLLER_AREA_CUTOFF)
    return NEUTRAL;

  // SCAN FOR BLUE
  cam.takeSnapshot(BLUE_GOAL);
  vex::vision::object blue_obj = cam.largestObject;
  int blue_area = blue_obj.width * blue_obj.height;
  int blue_y = blue_obj.centerY;

  if (cam.objectCount < 1 || blue_area < ROLLER_AREA_CUTOFF)
    return NEUTRAL;

  if (red_y > blue_y)
    return RED;
  else
    return BLUE;
}