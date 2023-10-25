#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/screen.hpp"

Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-17, -18,19}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{11,12,-13}

  // IMU Port
  ,15

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.25

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


pros::Motor cata(20, pros::E_MOTOR_GEARSET_18, false);
pros::Motor intake(14, pros::E_MOTOR_GEARSET_06, true);

pros::Rotation rotation(8);

pros::ADIDigitalIn lim('D');


pros::ADIDigitalOut wingspiss('A');
pros::ADIDigitalOut hangpiss('B');
pros::ADIDigitalOut blockerpiss('C');


pros::Controller con1 (pros::E_CONTROLLER_MASTER);



void cataTask();
bool cata_override = false;
bool state = true;


bool up = true;
bool down = false;
bool yes = true;
bool no = false;

int catapos = rotation.get_angle() / 100;

void cata_task_fn() {
  
  while (true) {
    
   
    int catapos = rotation.get_angle() / 100;

    if (!abs(catapos >= 31) && (state == false)) {
      // move catapult down until its reached loading position
      cata = 127;
      

    } else if (!cata_override && abs(catapos >= 31) ) {
      cata = 0;
      state = true;
    }
    

    

    pros::delay(10);
  }
}

void fire() {

  cata_override = true;
  cata = 127;
  
  pros::delay(500);
  cata_override = false;
  state = false;
 
}


void lower() {
    
    if (!(rotation.get_angle() >= 3100)) {
        cata = 127;
    } else {
        
    }

}


void wings(bool state) {

  wingspiss.set_value(state);
    
}

void blocker(bool state) {

  blockerpiss.set_value(state);
    
}








