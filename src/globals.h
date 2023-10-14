#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"


extern pros::Motor cata;
extern bool cata_override;
extern bool state;
extern bool cataMid;
extern bool up;
extern bool down;
extern bool yes;
extern bool no;
extern pros::Motor intake;
extern Drive chassis;
extern pros::Rotation rotation;
extern pros::ADIDigitalOut hang;
extern pros::ADIDigitalOut wings;
extern pros::ADIDigitalOut blocker;
extern pros::Controller con1;



extern void cata_task_fn();
extern void fire();
extern void lower();
extern void wingstate(bool);

extern pros::Controller con1;
