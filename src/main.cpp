#include "main.h"
#include"display/lvgl.h"
pros::Motor rightBack(12, pros::E_MOTOR_GEARSET_18, true);
pros::Motor rightFront(1, pros::E_MOTOR_GEARSET_18, true);

pros::Motor leftBack(21, pros::E_MOTOR_GEARSET_18, false);
pros::Motor leftFront(2, pros::E_MOTOR_GEARSET_18, false);


//lifter
const pros::Motor leftLift(10, pros::E_MOTOR_GEARSET_36, false);
const pros::Motor rightLift(18, pros::E_MOTOR_GEARSET_36, true);

//slide rotation
const pros::Motor slideRot(19, pros::E_MOTOR_GEARSET_36, true);
//loader rotation
const pros::Motor loaderRot(20, pros::E_MOTOR_GEARSET_36, true);

//main controller object
pros::Controller controller(pros::E_CONTROLLER_MASTER);
	#define DEADZONE 10

void initialize() {
	  slideRot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

//pid chassis controller.
static auto chassis = ChassisControllerBuilder()
    .withMotors(2, -1, -12, 21)
    
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 13.5_in}, imev5GreenTPR})
    .withGains(
        {0.0025, 0.0005, 0.0001}, // Distance controller gains
        {0.003, 0, 0.0001}, // Turn controller gains
        {0.00175, 0, 0.0001})  // Angle controller gains (helps drive straight)
        .withOdometry()
    .buildOdometry();

    /*
#ifndef PID
    //normal chassis controller.
    static auto chassis = ChassisControllerBuilder()
        .withMotors({19, 20}, {9, 10})
        // Green gearset, 4 in wheel diam, 11.5 in wheel track
        .withDimensions(AbstractMotor::gearset::green, {{3_in, 8.5_in}, imev5GreenTPR})

        .build();
  #endif
*/
static void strafeLeft(const unsigned int speed)
{
  rightFront.move(speed);
  rightBack.move(speed * -1);

  leftFront.move(speed * -1);
  leftBack.move(speed);
}

static void strafeRight(const unsigned int speed)
{
  rightFront.move(speed * -1);
  rightBack.move(speed);

  leftFront.move(speed);
  leftBack.move(speed * -1);
}


void disabled() {}


void competition_initialize() {}


//extern void externAuton();
void autonomous()
{
  //externAuton();
}

//a function that polls the controller for all mechanism related input like the loaders, slide, and arms.
static void pollLift()
{
    //if the left top buttton, move lift down
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      leftLift.move(127);
      rightLift.move(127);
    }
    else
    //if the right top button, move lift up
    if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      leftLift.move(-127);
      rightLift.move(-127);
    }
    else
    //if no life related input, make the lift hold it's position, (Hold brake mode)
    if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      leftLift.move(0);
      rightLift.move(0);
    }

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
      if(slideRot.get_position() < 1500)
      {
        slideRot.move(127);
      }
      else
      {
        slideRot.move(75);
      }
    }
    else
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      slideRot.move(-127);
    }
    else
    if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      if(slideRot.get_position() < 400)
      {
        std::cout<<"test"<<std::endl;
        slideRot.move(0);
      }
      else
        slideRot.move_velocity(0);
    }

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      loaderRot.move(127);
      if(slideRot.get_position() > 100)
      {
        slideRot.move(-127);
      }
    }
    else
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      loaderRot.move(-127);
      if(loaderRot.get_position() < -1300 && (slideRot.get_position() < 600))
      {
        slideRot.move(80);
      }
    }
    else
    if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        loaderRot.move_velocity(0);
    }
}
void opcontrol() 
{
	loaderRot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(label, "slide");
    lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t* labelA = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(labelA, "arm");
    lv_obj_align(labelA, NULL, LV_ALIGN_CENTER, 0, 100);
	while(true)
	{
		int Ch3 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int Ch1 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		int Ch4 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

		int LEFT_FRONT = Ch3 + Ch1 + Ch4;
		int LEFT_BACK = Ch3 + Ch1 - Ch4;

		int RIGHT_FRONT = Ch3 - Ch1 - Ch4;
		int RIGHT_BACK = Ch3 - Ch1 + Ch4;
 

    leftFront = LEFT_FRONT;
		leftBack = LEFT_BACK;

		rightFront = RIGHT_FRONT;
		rightBack = RIGHT_BACK;

		pollLift();

        std::string temp  = "slide rot : " + std::to_string(slideRot.get_position());
        lv_label_set_text(label, temp.c_str());
        temp = "arm rot : " + std::to_string(loaderRot.get_position());
        lv_label_set_text(labelA, temp.c_str());


        pros::Task::delay(10);
	}
}
