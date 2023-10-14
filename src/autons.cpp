#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "globals.h"
#include "main.h"
#include "pros/adi.h"
#include "pros/rtos.hpp"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 100;
const int SWING_SPEED = 100; //90



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 35, 0);//11, 0 ,20 ,0
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 38, 15);//5 .003 35, 15
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 50, 3, 250, 7, 250, 500); //100, 3, 500, 7, 500, 500
  chassis.set_exit_condition(chassis.swing_exit, 30, 3, 200, 7, 200, 500); // 100, 3, 500, 7, 500, 500
  chassis.set_exit_condition(chassis.drive_exit, 30, 100, 150, 150, 200, 500);//80, 50, 300, 150, 500, 500
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}



///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();
  pros::delay(10000);

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .

void wpL() {

  cataMid = true;
  // lower cata
  lower();

  //lower wings to lower intake
  pros::delay(500);


  // wing delay to come back up
  pros:: delay(200);

  

  // intake 
  intake.move_velocity(600);

  // move back to triball
  chassis.set_drive_pid(13, DRIVE_SPEED, true);
  chassis.wait_drive();

  // wait a lil for the triball to intake
  pros::delay(100);

  // turn around
  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();

  // move back towards matchloader
  chassis.set_drive_pid(20, 100);
  chassis.wait_drive();

   // swing to aim along matchloader
  chassis.set_swing_pid(ez::LEFT_SWING, 225, 110);
  chassis.wait_drive();

  // wing down to get preload
  pros::delay(1000);

  // move to get preload out
  chassis.set_drive_pid(10, 100);
  chassis.wait_drive();

  // swing towards goal 
  chassis.set_swing_pid(ez::LEFT_SWING, 270, 110);
  chassis.wait_drive();

  //intake out
  intake.move_velocity(-600);

  // drive into goal
  chassis.set_drive_pid(10, 100);
  chassis.wait_drive();

  //wing up
  pros::delay(500);

  //back out
  chassis.set_drive_pid(-5, 100);
  chassis.wait_drive();

  //turn towards center triball
  chassis.set_turn_pid(315, TURN_SPEED);
  chassis.wait_drive();

  // intake
  intake.move_velocity(600);

  // go to triball
  chassis.set_drive_pid(30, 100);
  chassis.wait_drive();

  // swing into triball
  chassis.set_swing_pid(LEFT_SWING, 360, TURN_SPEED);
  chassis.wait_drive();

  // slightly back up into it
  chassis.set_drive_pid(5, 100);
  chassis.wait_drive();


  // turn to goal
  chassis.set_turn_pid(180, 100);
  chassis.wait_drive();

  // wing out
  pros::delay(1000);

  // drive into goal
  chassis.set_drive_pid(30, 100);
  chassis.wait_drive();

  // back out
  chassis.set_drive_pid(-15, 100);
  chassis.wait_drive();





  

}

void halfwpR() {

  // lower cata
  lower();

  chassis.set_drive_pid(7, 90, true);
  chassis.wait_drive();

   chassis.set_turn_pid(25, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(1, 100, true);
  pros::delay(10);

   fire();

   chassis.set_drive_pid(-1, 90, true);
  chassis.wait_drive();

   chassis.set_swing_pid(ez::LEFT_SWING, 125, TURN_SPEED);
  chassis.wait_drive();

  
  chassis.set_drive_pid(25, 90, true);
  chassis.wait_drive();

   intake.move_relative(800, 600);
  pros::delay(500);

  chassis.set_drive_pid(-15, 90, true);
  chassis.wait_drive();

   chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

 // turn on intake
  intake.move_velocity(600);
  
  chassis.set_drive_pid(57, 60, true);
  chassis.wait_drive();
  
  // turn towards goal
  chassis.set_turn_pid(47, TURN_SPEED);
  chassis.wait_drive();

// drive to the line
  chassis.set_drive_pid(4, 100, true);
 pros::delay(200);
 
  // shoot
  fire();

  intake.move_velocity(600);

   chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-4, 90, true);
  chassis.wait_drive();


 intake.move_velocity(600);
   

  chassis.set_swing_pid(ez::RIGHT_SWING, -180, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, 50, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -334, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(36, 110, true);
  chassis.wait_drive();

  pros::delay(300);

  // shoot
  fire();


    


}

void halfwpL() {

  // lower cata
  lower();

  // move back to rollers
  chassis.set_drive_pid(-2, DRIVE_SPEED, true);
  chassis.wait_drive();

  pros::delay(100);

  // spin rollers
  intake.move_relative(-600, 600);
  pros::delay(500);

  //raise intake
  

 chassis.set_drive_pid(4.5, 100);
chassis.wait_drive();
 chassis.set_turn_pid(-9.5, TURN_SPEED);
  chassis.wait_drive();
 

  chassis.set_drive_pid(.5, 110);
chassis.wait_drive();


  

  fire();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(9, DRIVE_SPEED, true);
  chassis.wait_drive();

  



  pros::delay(200);

   // intake on
  intake.move_velocity(600);

   // turn to goal
  chassis.set_swing_pid(ez::RIGHT_SWING, -25, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(3, DRIVE_SPEED, true);
  chassis.wait_drive();

  // decimate stack
  
  pros::delay(1500);

 

  pros::delay(1000);

  // back away from line
  chassis.set_drive_pid(-8, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, -10, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(6, 110);
  pros::delay(40);
  

  fire();

   chassis.set_drive_pid(-11, DRIVE_SPEED, true);
  chassis.wait_drive();

 

  //raise intake
 

  //turn towards stack
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  // move over stack
  chassis.set_drive_pid(18, 110, true);
  chassis.wait_drive();

  // intake on
  intake.move_velocity(600);

  // decimate stack
  
  pros::delay(1400);

  // move away from line
  chassis.set_drive_pid(10, 110, true);
  chassis.wait_drive();

  // turn to goal
  chassis.set_turn_pid(-24, TURN_SPEED);
  chassis.wait_drive();

  // drive forwards and boost
  chassis.set_drive_pid(8, DRIVE_SPEED, true);
  pros::delay(200);
  
  // shoot
  fire();
 

  pros::delay(999899);

  


  chassis.set_drive_pid(-32, 110);
  chassis.wait_drive();

  chassis.set_turn_pid(95, TURN_SPEED);
  chassis.wait_drive();

   chassis.set_drive_pid(7, 110, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, 5, TURN_SPEED);
  chassis.wait_drive();

   chassis.set_drive_pid(43, 80, true);
  chassis.wait_drive();
  
  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

   
    pros::delay(100);


  fire();
  


  /*intake.move_velocity(600);

  chassis.set_drive_pid(-15, 90);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -1, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-35, 80);
  chassis.wait_drive();

 
  // turn to goal
  chassis.set_turn_pid(-32, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(44, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  // shoot
  fire();*/
 



  





 
 



}

void fivediskL(){
 // prime_cata();
 chassis.set_drive_pid(-2, DRIVE_SPEED, true);
 chassis.wait_drive();
 intake.move_relative(-100, -200);
 pros::delay(500);
 chassis.set_drive_pid(13, DRIVE_SPEED, true);//13
 chassis.wait_drive();
 chassis.set_swing_pid(ez::LEFT_SWING, 45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(60, DRIVE_SPEED, true);//60
 chassis.wait_drive();
 chassis.set_turn_pid(-45, TURN_SPEED);
 chassis.wait_drive();
 
 
  /*prime_cata();
  cata.move_velocity(100);
  pros::delay(500);
  prime_cata();
  cata.move_velocity(0);
  chassis.set_drive_pid(-1, DRIVE_SPEED, true);
 chassis.wait_drive();*/
 

  chassis.set_swing_pid(ez::LEFT_SWING, -135, TURN_SPEED);
  chassis.wait_drive();
  intake.move_velocity(200);
  
  chassis.set_drive_pid(-35, 65, true);
  chassis.wait_drive();
  
  chassis.set_drive_pid(30, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, -48, TURN_SPEED);
  chassis.wait_drive();
   intake.move_velocity(0);

   chassis.set_drive_pid(6, DRIVE_SPEED, true);

  
 
  
  
 
  /*prime_cata();
  cata.move_velocity(100);
  pros::delay(500);
  prime_cata();
  cata.move_velocity(0);*/
   chassis.set_drive_pid(-6, DRIVE_SPEED, true);
    chassis.wait_drive();
  intake.move_velocity(200);
  //chassis.set_swing_pid(ez::LEFT_SWING, -135, TURN_SPEED);
  //chassis.wait_drive();
  intake.move_velocity(200);
  chassis.set_drive_pid(-13, DRIVE_SPEED, true);
 chassis.wait_drive();
 pros::delay(900);
 
  
 
  
  
 
 chassis.set_turn_pid(-48, TURN_SPEED);
 chassis.wait_drive();
  intake.move_velocity(0);
 chassis.set_drive_pid(3, DRIVE_SPEED, true);

 
  /*prime_cata();
  cata.move_velocity(100);
  pros::delay(500);
  prime_cata();
  cata.move_velocity(0);*/
  

}

void skills() {

  // lower cata
  lower();

  // move back to rollers
  chassis.set_drive_pid(-2, DRIVE_SPEED, true);
  chassis.wait_drive();

  // spin rollers
  intake.move_relative(750, -600);
  pros::delay(300);

  // move off rollers
  chassis.set_drive_pid(3, 50, true);
  chassis.wait_drive();

  // turn to lone disk
  chassis.set_turn_pid(125, TURN_SPEED);
  chassis.wait_drive();

  //intake on
  intake.move_velocity(600);

  // Intake disc + get roller
  chassis.set_drive_pid(-33.5, 60, true);
  chassis.wait_until(-30);
  chassis.set_max_speed(50); 
  chassis.wait_drive();
  pros::delay(300);

  //drive away
  chassis.set_drive_pid(4, 50, true);
  chassis.wait_drive();

  // Swing to goal
  chassis.set_turn_pid( 0, TURN_SPEED);
  chassis.wait_drive();

  // Drive to goal
  chassis.set_drive_pid(55, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid( 3, TURN_SPEED);
  chassis.wait_drive();

  // Shoot
  fire();

  chassis.set_turn_pid( 0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

 

  // Get 3 on Barrier
  chassis.set_swing_pid(ez::RIGHT_SWING, -91, SWING_SPEED);
  chassis.wait_drive();

  // Intake 3 on barrier
  chassis.set_drive_pid(-34, 50, true);
  chassis.wait_drive();

  pros::delay(400);

  // Go to shoot
  chassis.set_drive_pid(29, 90, true);
  chassis.wait_drive();

  // Turn to goal
  chassis.set_turn_pid(3, TURN_SPEED);
  chassis.wait_drive();

  // Shoot
  fire();


  chassis.set_drive_pid(-2, 70, true);
  chassis.wait_drive();

  // Turn to first disk in line of 3
  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  // Move back to intake that disk
  chassis.set_drive_pid(-30, 70, true);
  chassis.wait_drive();

  // Turn to the rest
  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();

  // Intake the rest
  chassis.set_drive_pid(-29, 50, true);
  chassis.wait_drive();

  // Turn to goal
  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(11, 70, true);
  
  pros::delay(100);

  // Shoot
  fire();

  pros::delay(700);

  intake.move_velocity(-600);

  chassis.set_drive_pid(-22, 70, true);
  chassis.wait_drive();

  // Turn to last 3 on barrier
  chassis.set_swing_pid(ez::RIGHT_SWING, -181, SWING_SPEED);
  chassis.wait_drive();

  intake.move_velocity(600);
  // Intake line of 3 on barrier
  chassis.set_drive_pid(-55, 50, true);
  chassis.wait_drive();

  // Turn to goal 
  chassis.set_turn_pid(-96, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-3, 30, true);
  chassis.wait_drive();


  // Shoot
  fire();

  // Turn to 3 stack
  chassis.set_turn_pid(-50, TURN_SPEED);
  chassis.wait_drive();

  // Pistake Up
  

  pros::delay(200);

  // Drive over stack
  chassis.set_drive_pid(-25, 70, true);
  chassis.wait_drive();

  // Eat stack
 

  // Wait before swimming
  pros::delay(1000);

  chassis.set_turn_pid(-105, TURN_SPEED);
  chassis.wait_drive();


  // Drive to goal
  chassis.set_drive_pid(20, 70, true);
  chassis.wait_drive();

  // Turn to goal 
  chassis.set_turn_pid(-67, TURN_SPEED);
  chassis.wait_drive();

  // shoot
  fire();

  // turn straight
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  // pistake up
  

  // drive back to 3 stack by rollers
  chassis.set_drive_pid(-40, 70, true);
  chassis.wait_until(-20);
  chassis.set_max_speed(40); 
  chassis.wait_drive();

  // eat
  

  // wait before swimming
  pros::delay(800);

  // move back to roller
  chassis.set_drive_pid(-36, 80, true);
  chassis.wait_until(-25);
  intake.move_velocity(0);
  chassis.set_max_speed(90); 
  chassis.wait_drive();

  // spin rollers
  intake.move_relative(800, -600);
  pros::delay(500);

  // drive off roller
  chassis.set_drive_pid(30, DRIVE_SPEED, true);
  chassis.wait_drive();

  // turn to other roller
  chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();

  

  // drive back to roller 
  chassis.set_drive_pid(-25, 110, true);
  chassis.wait_until(-25);
  chassis.set_max_speed(100); 
  chassis.wait_drive();

  // spin rollers
  intake.move_relative(800, -600);
  pros::delay(500);

  
  // drive off roller
  chassis.set_drive_pid(5, 100, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-202, TURN_SPEED);
  chassis.wait_drive();


  chassis.set_drive_pid(69, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move_velocity(600);



  chassis.set_turn_pid(-179, TURN_SPEED);
  chassis.wait_drive();

  fire();

  chassis.set_swing_pid(ez::RIGHT_SWING, -271, SWING_SPEED);
  chassis.wait_drive();

  // Intake 3 on barrier
  chassis.set_drive_pid(-35, 60, true);
  chassis.wait_drive();

  pros::delay(200);

  // Go to shoot
  chassis.set_drive_pid(30, 70, true);
  chassis.wait_drive();

  // Turn to goal
  chassis.set_turn_pid(-178, TURN_SPEED);
  chassis.wait_drive();

  // Shoot
  fire();

  
  chassis.set_drive_pid(-2, 70, true);
  chassis.wait_drive();

  // Turn to first disk in line of 3
  chassis.set_turn_pid(-215, TURN_SPEED);
  chassis.wait_drive();

  // Move back to intake that disk
  chassis.set_drive_pid(-34, 70, true);
  chassis.wait_drive();

  // Turn to the rest
  chassis.set_turn_pid(-315, TURN_SPEED);
  chassis.wait_drive();

  // Intake the rest
  chassis.set_drive_pid(-33, 70, true);
  chassis.wait_drive();

  // Turn to goal
  chassis.set_turn_pid(-225, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(12, 70, true);
  

  pros::delay(200);


  // Shoot
  fire();

  intake.move_velocity(-600);

  chassis.set_drive_pid(-21, 80, true);
  chassis.wait_drive();

  // Turn to last 3 on barrier
  chassis.set_swing_pid(ez::RIGHT_SWING, -362, SWING_SPEED);
  chassis.wait_drive();

  intake.move_velocity(600);
  // Intake line of 3 on barrier
  chassis.set_drive_pid(-58, 50, true);
  chassis.wait_drive();

  // Turn to goal 
  chassis.set_turn_pid(-275, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-2, 90, true);
  chassis.wait_drive();


  // Shoot
  fire();

  // Turn to 3 stack
  chassis.set_turn_pid(-230, TURN_SPEED);
  chassis.wait_drive();

  // Pistake Up
 

  // Drive over stack
  chassis.set_drive_pid(-25, 80, true);
  chassis.wait_drive();

  // Eat stack
  

  // Wait before swimming
  pros::delay(800);

  chassis.set_turn_pid(-285, TURN_SPEED);
  chassis.wait_drive();


  // Drive to goal
  chassis.set_drive_pid(20, 70, true);
  chassis.wait_drive();

  // Turn to goal 
  chassis.set_turn_pid(-249, TURN_SPEED);
  chassis.wait_drive();

  // shoot
  fire();

  // Turn to 3 stack
  chassis.set_turn_pid(-273, TURN_SPEED);
  chassis.wait_drive();

  // Pistake Up
  

  // Drive over stack
  chassis.set_drive_pid(-40, 80, true);
  chassis.wait_drive();

  // Eat stack
  

  // Wait before swimming
  pros::delay(1000);
  
  // Drive to goal
  chassis.set_drive_pid(40, 70, true);
  chassis.wait_drive();

  // Turn to goal 
  chassis.set_turn_pid(-247, TURN_SPEED);
  chassis.wait_drive();

  // shoot
  fire();


  chassis.set_turn_pid(-285, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-66, 110, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-317, TURN_SPEED);
  chassis.wait_drive();


 



}


void nothing(){

}