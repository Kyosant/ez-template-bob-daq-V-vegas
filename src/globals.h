#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"



extern class robotWings wingy;
extern pros::Motor cata;
extern bool cata_override;
extern bool state;
extern bool up;
extern bool down;
extern bool yes;
extern bool no;
extern pros::Motor intake;
extern Drive chassis;
extern pros::Rotation rotation;
extern pros::ADIDigitalOut hangpiss;
extern pros::ADIDigitalOut wingspiss;
extern pros::ADIDigitalOut blockerpiss;
extern pros::Controller con1;





extern void cata_task_fn();
extern void fire();
extern void lower();
extern void wings(bool state);
extern void blocker(bool state);

extern pros::Controller con1;
