#include "competition/opcontrol.h"
#include "robot-config.h"
#include "tuning.h"
#include "testing.h"
#include "competition/autonomous_clu.h"
#define TESTING_BUILD

void print_to_screen()
{
  static timer tmr;

  if (tmr.time() > 1)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(0, 0);
    Brain.Screen.print("Battery: %d%", Brain.Battery.capacity());
    Brain.Screen.print("Flywheel Temp: %2f F", flywheel.temperature());
    Brain.Screen.setCursor(1, 0);
    if (flywheel.installed())
      Brain.Screen.print("Flywheel Connected");
    else
      Brain.Screen.print("WARNING || FLYWHEEL DISCONNECTED!");
    Brain.Screen.print("Flywheel RPM: %2f", flywheel_sys.getRPM());
    tmr.reset();
  }
}

#define SHOTLENGTH 100
#define DELAYLENGTH 300
void tripleshot()
{
  for (int i = 0; i < 3; i++)
  {
    intake.spin(fwd, 12, volt);
    vexDelay(SHOTLENGTH);
    intake.stop();
    vexDelay(DELAYLENGTH);
  }
}

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */

void opcontrol()
{

  // Initialization
  printf("starting\n");
  fflush(stdout);

  // Intake - R1
  main_controller.ButtonR1.pressed([]()
                                   { intake.spin(reverse, 12, volt); });
  main_controller.ButtonR1.released([]()
                                    { intake.stop(); });

  // Shoot - R2
  main_controller.ButtonR2.pressed([]()
                                   { intake.spin(fwd, 9.5, volt); });
  main_controller.ButtonR2.released([]()
                                    { intake.stop(); });
#ifdef TESTING_BUILD
  main_controller.ButtonRight.pressed([]()
                                      { flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() + 100); });
  main_controller.ButtonLeft.pressed([]()
                                     { flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() - 100); if (flywheel_sys.getDesiredRPM() <=0){flywheel_sys.stop();} });

#endif

  // Flap - Y
  main_controller.ButtonY.pressed([]()
                                  {
    static bool flapUp = false;
    flapUp = !flapUp;
    flapup_solenoid.set(flapUp); });

  // Single Shot - L2
  main_controller.ButtonL2.pressed([]()
                                   {
    intake.spin(fwd, 12, volt);
    vexDelay(SHOTLENGTH);
    intake.stop();
    vexDelay(DELAYLENGTH); });

  // Flywheel set RPM
  flywheel_sys.spinRPM(4000);
  odometry_sys.end_async();

  flap_up();
  timer tmr;

  GraphDrawer setpt_graph(Brain.Screen, 30, "RPM", "Time", vex::red, true, 0, 4000);
  GraphDrawer rpm_graph(Brain.Screen, 30, "RPM", "Time", vex::red, true, 0, 4000);

  VisionAimCommand visaim;
  int i = 0;
  double time = 0.0;
  // Periodic
  while (true)
  {
    // print_to_screen();
    i++;
    if (i % 5 == 0)
    {
      main_controller.Screen.setCursor(0, 0);
      main_controller.Screen.clearScreen();
      main_controller.Screen.print("fw rpm: %f", flywheel_sys.getRPM());
      main_controller.Screen.setCursor(2, 0);
      main_controller.Screen.print("fw temp: %.1ff", flywheel.temperature(vex::fahrenheit));
      main_controller.Screen.setCursor(4, 0);
      main_controller.Screen.print("bat fw : %.2fv %.2fv", Brain.Battery.voltage(vex::volt), flywheel.voltage(volt));
    }
    time += 0.02;
    setpt_graph.add_sample(Vector2D::point_t{.x = time, .y = flywheel_sys.getDesiredRPM()});
    rpm_graph.add_sample(Vector2D::point_t{.x = time, .y = flywheel_sys.getRPM()});

    // ========== DRIVING CONTROLS ==========
    if (!main_controller.ButtonX.pressing())
      drive_sys.drive_tank(main_controller.Axis3.position() / 100.0, main_controller.Axis2.position() / 100.0);
    else
      visaim.run();
    // drive_sys.drive_arcade(main_controller.Axis3.position()/100.0, main_controller.Axis1.position()/100.0);

    // ========== MANIPULATING CONTROLS ==========
    if (main_controller.ButtonL1.pressing() && main_controller.ButtonL2.pressing() && main_controller.ButtonR1.pressing() && main_controller.ButtonR2.pressing())
    {
      endgame_solenoid.set(true);
    }

    if (main_controller.ButtonDown.pressing())
    {
      flywheel_sys.stop();
    }
    else if (main_controller.ButtonUp.pressing())
    {
      flywheel_sys.spinRPM(4000);
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========

#ifdef TESTING_BUILD
    auto screen = Brain.Screen;
    screen.clearScreen();
    setpt_graph.draw(20, 20, 360, 200);
    rpm_graph.draw(20, 20, 400, 200);
    screen.printAt(390, 100, "sp: %.0f", flywheel_sys.getDesiredRPM());
    screen.printAt(390, 120, "pv: %.0f", flywheel_sys.getRPM());
    screen.render();
#endif
    vexDelay(20);
  }
}