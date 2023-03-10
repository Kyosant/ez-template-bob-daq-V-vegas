#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"


extern pros::Motor cata;
extern bool cata_override;
extern bool state;
extern pros::Motor intake;
extern Drive chassis;
extern pros::ADIDigitalOut Endgame;
extern pros:: ADIDigitalIn limit;
extern pros::ADIDigitalOut Bands;
extern pros::ADIDigitalOut Pistake;
extern pros::Controller con1;


extern void cata_task_fn();
extern void fire();
extern void lower();
extern pros::Controller con1;
