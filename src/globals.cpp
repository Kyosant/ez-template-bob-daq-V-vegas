#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-1,-2,3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{6, 7,-12}

  // IMU Port
  ,11

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.666

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);


pros::Motor cata(9, pros::E_MOTOR_GEARSET_36, false);
pros::Motor intake(4, pros::E_MOTOR_GEARSET_06, true);

pros::ADIDigitalIn limit('A');
pros::ADIDigitalOut Endgame('B');
pros::ADIDigitalOut Bands('C');
pros::ADIDigitalOut Pistake('D');


pros::Controller con1 (pros::E_CONTROLLER_MASTER);

void cataTask();
bool cata_override = false;
bool state = true;

void cata_task_fn() {
  
  while (true) {

    if ((limit.get_value() == false) && state == false) {
      // move catapult down until its reached loading position
      cata = 127;

    } else if (!cata_override && limit.get_value()) {
      cata = 0;
      state = true;
    }

    pros::delay(10);
  }
}

void fire() {
  cata_override = true;
  cata = 127;
  pros::delay(300);
  cata_override = false;
  state = false;
}

void lower() {
    
    if (!limit.get_value()) {
        cata = 127;
    } else {
        
    }

}