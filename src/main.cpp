#include "main.h"
#include "math.h"
#include <limits>

/*=============
** PORT DECLARATIONS
=============*/
#define LEFT_WHEEL_FRONT_PORT 13
#define LEFT_WHEEL_BACK_PORT 17
#define RIGHT_WHEEL_FRONT_PORT 8
#define RIGHT_WHEEL_BACK_PORT 7

#define FEEDER_MIDDLE_PORT 18
#define FEEDER_TOP_PORT 6

#define LEFT_INTAKE_PORT 16
#define RIGHT_INTAKE_PORT 10

#define IMU_PORT 19

#define VISION_PORT 3

#define LEFT_ENCODER_TOP 8
#define LEFT_ENCODER_BOTTOM 7
#define CENTER_ENCODER_TOP 6
#define CENTER_ENCODER_BOTTOM 5
#define RIGHT_ENCODER_TOP 4
#define RIGHT_ENCODER_BOTTOM 3

#define ANALOG_SENSOR_PORT1 1
#define ANALOG_SENSOR_PORT2 2
#define ANALOG_SENSOR_PORT3 9

/*=============
** COMPONENT DECLARATION
=============*/
pros::Motor left_wheel_front (LEFT_WHEEL_FRONT_PORT);
pros::Motor left_wheel_back (LEFT_WHEEL_BACK_PORT);
pros::Motor right_wheel_front (RIGHT_WHEEL_FRONT_PORT);
pros::Motor right_wheel_back (RIGHT_WHEEL_BACK_PORT);

pros::Motor feeder_middle (FEEDER_MIDDLE_PORT);
pros::Motor feeder_top (FEEDER_TOP_PORT);

pros::Motor left_intake (LEFT_INTAKE_PORT);
pros::Motor right_intake (RIGHT_INTAKE_PORT);

pros::Imu inertial (IMU_PORT);

pros::ADIEncoder left_encoder (LEFT_ENCODER_TOP, LEFT_ENCODER_BOTTOM, false);
pros::ADIEncoder right_encoder (RIGHT_ENCODER_TOP, RIGHT_ENCODER_BOTTOM, false);
pros::ADIEncoder center_encoder (CENTER_ENCODER_TOP, CENTER_ENCODER_BOTTOM, false);

pros::Vision vision_sensor (VISION_PORT);

pros::Controller master (pros::E_CONTROLLER_MASTER);

pros::ADIAnalogIn ball_limit_switch (ANALOG_SENSOR_PORT2);
pros::ADIAnalogIn ball_limit_switch2 ({{ANALOG_SENSOR_PORT3, 'A'}});
pros::ADIAnalogIn goal_limit_switch (ANALOG_SENSOR_PORT1);

#define SIDE 1

#define PI 3.1415926

#define KPBASE 0.11
#define KPBASETURN 0.1

#define intake_speed 127
#define conveyer_speed 127

/*=============
** GLOBAL POSITION/ANGLE VARIABLES USED FOR POSITION TRACKING
=============*/
float pos_x = 0;
float pos_y = 0;

float angle_ = 0;

//Change in position is calculated based on difference between previous encoder based position (For position tracking everywhere)
float prev_encoder_fwd_rev = 0;
float prev_encoder_left_right = 0;
float prev_angle = 0;

//Used to see if bot is relatively close to goal position (fine-tuned value)
float close_turn = 35;
float close_move = 400;

bool wait_special = false;

bool topEngaged = false;
bool middleEngaged = false;
int motorVal = 0;
int middleMotorVal = 0;

long topEngagedTime;



void stopHold(){
	left_wheel_front.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_wheel_back.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_wheel_front.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_wheel_back.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	left_wheel_front.move(0);
	left_wheel_back.move(0);
	right_wheel_front.move(0);
	right_wheel_back.move(0);
}


void stopCoast(){
	left_wheel_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_wheel_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_wheel_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_wheel_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	left_wheel_front.move(0);
	left_wheel_back.move(0);
	right_wheel_front.move(0);
	right_wheel_back.move(0);
}

void turnOffIntake(){
	left_intake.move(0);
	right_intake.move(0);
}


void drive(float goal_x, float goal_y, float goal_heading, float move_speed, float turn_speed, float position_tolerance, float angle_tolerance, bool store_our, bool poop, bool flag_change=false, bool fast_poop=false, bool extra_fast_poop=false){
  //Angle bounds used to calculate when to stop turning (ex. +-3)
	int upperAngleBound = goal_heading + angle_tolerance;
	int lowerAngleBound = goal_heading - angle_tolerance;

	long begin_time = pros::millis();

	//Special conditions (If the angle is at 0 for example)
	bool specialDown = false;
	bool specialUp = false;
	if(lowerAngleBound < 0){
		lowerAngleBound = lowerAngleBound + 360;
		specialDown = true;}
	if(upperAngleBound > 360){
		upperAngleBound = upperAngleBound - 360;
		specialUp = true;}

		if(store_our == true && topEngaged == false){
			if(ball_limit_switch.get_value() < 80){
				feeder_middle.move(-conveyer_speed);
				feeder_top.move(-90);
			}
			else{
				if(ball_limit_switch2.get_value() < 1800 && middleEngaged == false){
					feeder_middle.move(-conveyer_speed/10*8);
					feeder_top.move(0);
				}
				else{
					feeder_middle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
					feeder_middle.move(0);
					feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
					feeder_top.move(0);
					middleEngaged = true;
				}
			}
		}
		else if(poop == true){
			feeder_middle.move(-conveyer_speed/10*6.6);
			feeder_top.move(conveyer_speed/10*6.5);
		}
		else if(fast_poop == true){
			feeder_middle.move(-conveyer_speed/10*8.7);
			feeder_top.move(conveyer_speed/10*7.7);
		}
		else if(extra_fast_poop == true){
			feeder_middle.move(-conveyer_speed);
			feeder_top.move(conveyer_speed);
		}

	bool flag = false;

	long goalReachedTime = 0;
	int goalReachedCount = 0;

	/*Main loop that runs during a drive function
		Specific conditions keep the drive function running until goal pos and heading is reached*/
	while(
		(sqrt(pow(goal_x-pos_x, 2)+pow(goal_y-pos_y, 2))>position_tolerance ||
		(specialDown == true && ((angle_ <  lowerAngleBound && angle_ > goal_heading+180) || (angle_ > upperAngleBound && angle_ < goal_heading+180))) ||
		(specialUp == true && ((angle_ > upperAngleBound && angle_ < goal_heading-180) || (angle_ < lowerAngleBound && angle_ > goal_heading-180))) ||
		((specialUp == false && specialDown == false) && (angle_ > upperAngleBound || angle_ < lowerAngleBound))) && flag == false
	){
		if(ball_limit_switch.get_value() > 80 && poop == false && topEngaged == false){
			topEngaged = true;
			topEngagedTime = pros::millis();
			motorVal = feeder_top.get_position()-130;
		}

		if(topEngaged == true){
			if(feeder_top.get_position() < motorVal){
				feeder_top.move(60 * (abs(feeder_top.get_position()-motorVal)/50 + 0.4));
			}
			else{
				feeder_top.move(-60 * (abs(feeder_top.get_position()-motorVal)/50 + 0.4));
			}
		}

		if(topEngaged == true && store_our == true && middleEngaged == false && pros::millis() - topEngagedTime > 150){
			if(ball_limit_switch2.get_value() < 1700){
				feeder_middle.move(-conveyer_speed/10*9);
			}
			else{
				middleEngaged = true;
				middleMotorVal = feeder_middle.get_position();
			}
		}

		if(middleEngaged == true){
			if(feeder_middle.get_position() < middleMotorVal){
				feeder_middle.move(40 * (abs(feeder_middle.get_position()-middleMotorVal)/50 + 0.4));
			}
			else{
				feeder_middle.move(-40 * (abs(feeder_middle.get_position()-middleMotorVal)/50 + 0.4));
			}
		}

		// Position tracking stuff
		float curr_encoder_fwd_rev = (right_encoder.get_value() + left_encoder.get_value()) / 2;
		float curr_encoder_left_right = center_encoder.get_value();

		angle_ = inertial.get_heading();
		if(angle_ > 360) angle_ = 0;
		if(angle_ < 0) angle_ = angle_+360;
		pos_x = pos_x - (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * -sin(angle_*PI/180)) + ((curr_encoder_left_right-prev_encoder_left_right) * -cos(angle_*PI/180)));
		pos_y = pos_y + (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * cos(angle_*PI/180)) - ((curr_encoder_left_right-prev_encoder_left_right) * sin(angle_*PI/180)));
		prev_encoder_fwd_rev = curr_encoder_fwd_rev;
		prev_encoder_left_right = curr_encoder_left_right;

		pros::lcd::set_text(1, std::to_string(pos_x));
		pros::lcd::set_text(2, std::to_string(pos_y));


		//Calculating reference angle
		int reference_angle = atan((goal_x-pos_x)/(goal_y-pos_y)) * 180 / PI;
		if((goal_y-pos_y) < 0) reference_angle = 180 + reference_angle;
		if(reference_angle < 0) reference_angle = 360 + reference_angle;

		//Calcualating difference in angles
		float difference = angle_ - reference_angle;
		if(difference < 0) difference = 360 + difference;
		difference = 360 - difference;
		float turn_difference = angle_-goal_heading;
		if(turn_difference > 180) turn_difference = 360-turn_difference;
		if(turn_difference < -180) turn_difference = 360+turn_difference;
		if(turn_difference < 0 && turn_difference > -180) turn_difference = -turn_difference;

		//Calculating difference for proportional calculations (PID allows smoother stops and error correcting)
		int up_down_difference = goal_y-pos_y;
		int left_right_difference = goal_x-pos_x;

		//Motor speed changing vaiables (Moves the robot regardless of heading and relative to field)
		float actual_up_down = 0;
		float actual_left_right = 0;
		float actual_turn = 0;

		if(up_down_difference < 0) up_down_difference = -up_down_difference;
		if(left_right_difference < 0) left_right_difference = -left_right_difference;

		if(sqrt(pow(goal_x-pos_x, 2)+pow(goal_y-pos_y, 2)) > close_move){
			actual_up_down = (cos(difference * PI / 180.0)*move_speed);
			actual_left_right = (sin(difference * PI / 180.0)*move_speed);
		}
		else{
			actual_up_down = (cos(difference * PI / 180.0)*move_speed) * (sqrt(pow(goal_x-pos_x, 2)+pow(goal_y-pos_y, 2))/close_move + KPBASE);
			actual_left_right = (sin(difference * PI / 180.0)*move_speed) * (sqrt(pow(goal_x-pos_x, 2)+pow(goal_y-pos_y, 2))/close_move + KPBASE);
		}

		//Makes sure the speed values are within the motor input value spectrum (-12000 mV and 12000 mV)
		if(actual_up_down > move_speed || actual_up_down < -move_speed)
			actual_up_down = (cos(difference * PI / 180.0)*move_speed);
		if(actual_left_right > move_speed || actual_left_right < -move_speed)
			actual_left_right = (sin(difference * PI / 180.0)*move_speed);

			//Spins in the directions which will allow bot to complete turn fastest
			if(turn_difference > 180) turn_difference = 360-turn_difference;

			//Slows down if close to goal heading and stays fast if it is away
			if(turn_difference < close_turn){
				actual_turn = (turn_speed*((turn_difference/(close_turn))+KPBASETURN));
			}
			else{
				actual_turn = turn_speed;
			}

			/*pros::lcd::set_text(1, std::to_string(up_down_difference));
			pros::lcd::set_text(2, std::to_string(left_right_difference));
			pros::lcd::set_text(3, std::to_string(turn_difference));

			pros::lcd::set_text(4, std::to_string(actual_up_down));
			pros::lcd::set_text(5, std::to_string(actual_left_right));*/

			/*Special conditions if angle bounds are less than 0 or greater than 360
				Neccesary for proper turning and calculation*/
			if((goal_heading > angle_+180) || (goal_heading > angle_-180 && angle_ > 180 && goal_heading < 180) ||
				(goal_heading < angle_ && angle_-goal_heading < 180)) actual_turn = -actual_turn;
			if(!(((upperAngleBound < angle_ || lowerAngleBound > angle_) && (specialUp == false && specialDown == false)) ||
			((specialDown == true) && (lowerAngleBound > angle_ || (upperAngleBound < angle_ && (!angle_ > lowerAngleBound)))) ||
			((specialUp == true) && (upperAngleBound < angle_ || (lowerAngleBound > angle_ && (!angle_ < upperAngleBound)))))) actual_turn = 0;


    //Applying final values to motors for motion
		left_wheel_front.move(actual_up_down + actual_left_right + actual_turn);
		left_wheel_back.move(actual_up_down - actual_left_right + actual_turn);
		right_wheel_front.move(-actual_up_down + actual_left_right + actual_turn);
		right_wheel_back.move(-actual_up_down - actual_left_right + actual_turn);

		if(goal_limit_switch.get_value() < 1700 && pros::millis()-goalReachedTime > 10){
			goalReachedTime = pros::millis();
			goalReachedCount += 1;
		}

		if(goal_limit_switch.get_value() < 1700 && goal_heading != 92 && goalReachedCount > 3){
			if(flag_change == true){
				flag = true;

				stopHold();
			}
		}
		else if(goal_limit_switch.get_value() < 1700 && goal_heading == 92 && goalReachedCount > 3){
			if(flag_change == true && pos_x > 850){
				flag = true;

				bool small_flag = false;

				left_wheel_front.move(-80);
				left_wheel_back.move(-80);
				right_wheel_front.move(80);
				right_wheel_back.move(80);

				long small_begin_time = pros::millis();

				float saved_x = pos_x;

				while(small_flag == false){
					// Position tracking stuff
					float curr_encoder_fwd_rev = (right_encoder.get_value() + left_encoder.get_value()) / 2;
					float curr_encoder_left_right = center_encoder.get_value();

					angle_ = inertial.get_heading();
					if(angle_ > 360) angle_ = 0;
					if(angle_ < 0) angle_ = angle_+360;
					pos_x = pos_x - (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * -sin(angle_*PI/180)) + ((curr_encoder_left_right-prev_encoder_left_right) * -cos(angle_*PI/180)));
					pos_y = pos_y + (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * cos(angle_*PI/180)) - ((curr_encoder_left_right-prev_encoder_left_right) * sin(angle_*PI/180)));
					prev_encoder_fwd_rev = curr_encoder_fwd_rev;
					prev_encoder_left_right = curr_encoder_left_right;

					if(pros::millis()-small_begin_time > 75){
						small_flag = true;
					}

					pros::delay(1);
				}
				stopCoast();
			}
		}

		/**/

		//Saves resources
    pros::delay(1);
	}

	feeder_middle.move_voltage(0);
	feeder_top.move_voltage(0);
}

void turnOnIntake(){
	left_intake.move(intake_speed);
	right_intake.move(-intake_speed);
}


void turnOnIntakeTwoThirds(){
	left_intake.move(intake_speed/3*2);
	right_intake.move(-intake_speed/3*2);
}

void turnOnIntakeHalf(){
	left_intake.move(intake_speed/2);
	right_intake.move(-intake_speed/2);
}

void turnOnIntakeOneFourth(){
	left_intake.move(intake_speed/4);
	right_intake.move(-intake_speed/4);
}


void reverseIntake(){
	left_intake.move(-intake_speed);
	right_intake.move(intake_speed);
}

void reverseIntakeHalf(){
	left_intake.move(-intake_speed/2.5);
	right_intake.move(intake_speed/2.5);
}

void reverseIntakeOneThird(){
	left_intake.move(-intake_speed/3);
	right_intake.move(intake_speed/3);
}

void reverseIntakeOneFourth(){
	left_intake.move(-intake_speed/4);
	right_intake.move(intake_speed/4);
}

void reverseIntakeOneFifth(){
	left_intake.move(-intake_speed/5);
	right_intake.move(intake_speed/5);
}


void scoreAndStore(float ball){
	if(ball == 1){
		feeder_top.move(-conveyer_speed);
		feeder_middle.move(-conveyer_speed/10*8);

		left_wheel_front.move(20);
		left_wheel_back.move(20);
		right_wheel_front.move(-20);
		right_wheel_back.move(-20);
	}
	else if(ball == 2){
		feeder_top.move(-conveyer_speed);
		feeder_middle.move(-conveyer_speed/10*8);

		left_wheel_front.move(30);
		left_wheel_back.move(30);
		right_wheel_front.move(-30);
		right_wheel_back.move(-30);
	}
	else if(ball == 3){
		feeder_top.move(-conveyer_speed);
		feeder_middle.move(-conveyer_speed);
	}
	else if(ball == 4){
		feeder_top.move(-conveyer_speed);
		feeder_middle.move(-conveyer_speed/10*8);

		left_wheel_front.move(20);
		left_wheel_back.move(20);
		right_wheel_front.move(-20);
		right_wheel_back.move(-20);
	}
	else if(ball == 5){
		feeder_top.move(-conveyer_speed);
		feeder_middle.move(-conveyer_speed/2);

		left_wheel_front.move(20);
		left_wheel_back.move(20);
		right_wheel_front.move(-20);
		right_wheel_back.move(-20);
	}

	if(ball == 1){
		bool flag = false;
		long begin_time = pros::millis();

		bool phase1 = false;
		bool phase2 = false;
		bool phase3 = false;

		int motorVal;
		bool setMotorVal = false;

		while (flag == false) {
			if(ball_limit_switch2.get_value() > 1800 && phase1 == false){
				phase1 = true;
			}
			if(phase1 == true && ball_limit_switch2.get_value() < 1650 && phase2 == false && pros::millis()-begin_time > 200){
				phase2 = true;
				if(setMotorVal == false){
					feeder_middle.move(-conveyer_speed/1.8);
				}
			}
			if(phase2 == true && ball_limit_switch.get_value() > 180 && phase3 == false){
				phase3 = true;
			}
			if(phase2 == true && ball_limit_switch2.get_value() > 1650 && setMotorVal == false && pros::millis()-begin_time > 150){
				feeder_middle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				feeder_middle.move(0);
				motorVal = feeder_middle.get_position()+230;
				setMotorVal = true;
			}
			if(setMotorVal == true){
				if(feeder_middle.get_position() < motorVal){
					feeder_middle.move(127);
				}
				else{
					feeder_middle.move(-127);
				}
			}
			if(pros::millis()-begin_time > 600){
				turnOffIntake();
			}
			if(pros::millis() - begin_time > 760){
				flag = true;
			}
			pros::delay(1);
		}
		feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		feeder_top.move(0);
	}
	else if(ball == 2){
		bool flag = false;
		long begin_time = pros::millis();

		bool phase1 = false;
		bool phase2 = false;
		bool phase3 = false;

		int motorVal;
		bool setMotorVal = false;

		turnOffIntake();

		while (flag == false) {
			if(ball_limit_switch2.get_value() > 1800 && phase1 == false){
				phase1 = true;
			}
			if(phase1 == true && ball_limit_switch2.get_value() < 1650 && phase2 == false && pros::millis()-begin_time > 200){
				phase2 = true;
			}
			if(phase2 == true && ball_limit_switch.get_value() > 180 && phase3 == false){
				if(setMotorVal == false){
					feeder_middle.move(-conveyer_speed/1.5);
				}
				phase3 = true;
			}
			if(phase2 == true && ball_limit_switch2.get_value() > 1650 && setMotorVal == false && pros::millis()-begin_time > 100){
				feeder_middle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				feeder_middle.move(0);
				motorVal = feeder_middle.get_position()+270;
				setMotorVal = true;
			}
			if(setMotorVal == true){
				if(feeder_middle.get_position() < motorVal){
					feeder_middle.move(127);
				}
				else{
					feeder_middle.move(-127);
				}
			}
			if(pros::millis()-begin_time < 340 && pros::millis()-begin_time > 180){
				turnOnIntake();
			}
			else if(pros::millis()-begin_time > 340){
				turnOffIntake();
			}
			if(pros::millis() - begin_time > 800){
				flag = true;
			}
			pros::delay(1);
		}
		feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		feeder_top.move(0);
	}
	else if(ball == 3){
		bool flag = false;
		long begin_time = pros::millis();

		bool phase1 = false;
		bool phase2 = false;
		bool phase3 = false;

		int motorVal;
		bool setMotorVal = false;

		bool startedScoring = false;
		long startedScoringTime;

		int count = 0;

		bool engaged = false;

		turnOffIntake();

		while (flag == false) {

			if(ball_limit_switch2.get_value() > 1800 && phase1 == false){
				phase1 = true;
			}
			if(phase1 == true && ball_limit_switch2.get_value() < 1650 && phase2 == false && pros::millis()-begin_time > 200){
				phase2 = true;
				if(setMotorVal == false){
					feeder_middle.move(-conveyer_speed/1.8);
				}
				startedScoring = true;
				startedScoringTime = pros::millis();
			}
			if(phase2 == true && ball_limit_switch.get_value() > 180 && phase3 == false){
				phase3 = true;
			}
			if(phase2 == true && ball_limit_switch2.get_value() > 1650 && setMotorVal == false && pros::millis()-begin_time > 300){
				feeder_middle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				feeder_middle.move(0);
				motorVal = feeder_middle.get_position()+300;
				setMotorVal = true;
			}
			if(startedScoring == true && pros::millis()-startedScoringTime < 500){
				if(setMotorVal == true){
					if(feeder_middle.get_position() < motorVal){
						feeder_middle.move(conveyer_speed);
					}
					else{
						feeder_middle.move(-conveyer_speed);
					}
				}
			}
			else if(startedScoring == true && pros::millis()-startedScoringTime > 500){
				feeder_middle.move(-conveyer_speed);
				feeder_top.move(conveyer_speed);
			}

			if(pros::millis()-begin_time > 50){
				turnOnIntake();
			}

			if(startedScoring == true && phase3 == true && ball_limit_switch2.get_value() > 1700 && engaged == false && pros::millis()-startedScoringTime > 550){
				engaged = true;
				count += 1;
			}

			if(startedScoring == true && phase3 == true && ball_limit_switch2.get_value() < 1700 && engaged == true && pros::millis()-startedScoringTime > 550){
				engaged = false;
			}

			if(count >= 2){
				bool small_flag = false;
				long small_start_time = pros::millis();
				while(small_flag == false){
					if(pros::millis()-small_start_time > 60){
						small_flag = true;
					}
					if(engaged == false){
						small_flag = true;
					}
				}
				flag = true;
			}
			else if(pros::millis()-begin_time > 2200){
				flag = true;
			}
			pros::delay(1);
		}
		feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		feeder_top.move(0);

	}
	else if(ball == 4){
		bool flag = false;
		long begin_time = pros::millis();

		bool phase1 = false;

		while (flag == false) {
			if(ball_limit_switch2.get_value() > 1700 && pros::millis() - begin_time > 150){
				feeder_middle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				feeder_middle.move(0);

				phase1 = true;

				feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				feeder_top.move(0);
			}
			if(pros::millis() - begin_time > 600){
				flag = true;
			}
		}

		feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		feeder_top.move(0);
		turnOffIntake();
	}
	else if(ball == 5){
		bool flag = false;
		long begin_time = pros::millis();

		bool stored = false;

		while (flag == false) {

			if(ball_limit_switch2.get_value() > 1700 && stored == false){
				feeder_middle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				feeder_middle.move(0);

				bool small_flag = false;
				long small_begin_time = pros::millis();

				while(small_flag == false){
					if(pros::millis()-small_begin_time > 250){
						small_flag = true;
					}

					// Position tracking stuff
					float curr_encoder_fwd_rev = (right_encoder.get_value() + left_encoder.get_value()) / 2;
					float curr_encoder_left_right = center_encoder.get_value();

					angle_ = inertial.get_heading();
					if(angle_ > 360) angle_ = 0;
					if(angle_ < 0) angle_ = angle_+360;
					pos_x = pos_x - (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * -sin(angle_*PI/180)) + ((curr_encoder_left_right-prev_encoder_left_right) * -cos(angle_*PI/180)));
					pos_y = pos_y + (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * cos(angle_*PI/180)) - ((curr_encoder_left_right-prev_encoder_left_right) * sin(angle_*PI/180)));
					prev_encoder_fwd_rev = curr_encoder_fwd_rev;
					prev_encoder_left_right = curr_encoder_left_right;
				}

				feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				feeder_top.move(0);
				stored = true;
			}
			if(pros::millis() - begin_time > 400){
				turnOffIntake();
			}
			if(pros::millis() - begin_time > 450){
				flag = true;
			}
		}

		feeder_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		feeder_top.move(0);
	}
	feeder_middle.move(0);
	feeder_top.move(0);
	topEngaged = false;
	middleEngaged = false;
	turnOffIntake();
}


void initialize() {
	//Initializing the screen for writing purposes
	pros::lcd::initialize();

	//Resetting inertial sensor to work with current heading
	inertial.reset();
	pros::delay(3500);

	inertial.tare();

	//Calibrating sensors
	ball_limit_switch.calibrate();
	goal_limit_switch.calibrate();

	pros::delay(3500);
}

void disabled() {}

void competition_initialize() {

}

void autonomous(){
  long initial_time = pros::millis();

	turnOnIntake();

	drive(0, 250, 0, 127, 40, 100, 6, false, false);

	pros::delay(200);

	drive(-720, 300, 237, 127, 127, 40, 6, true, false);

	drive(-1100, 100, 236, 120, 60, 1, 3, true, false, true);

	scoreAndStore(1);

	reverseIntakeOneThird();

	drive(-680, 410, 230, 127, 127, 70, 6, false, false);

	turnOnIntake();

	drive(-570, 520, 231, 127, 127, 100, 6, false, false);

	drive(-550, 620, 284, 127, 100, 250, 10, false, false, false, true);

	drive(-790, 890, 280, 127, 40, 60, 6, false, false, false, true);

	stopHold();
	feeder_top.move(-conveyer_speed);
	feeder_middle.move(-conveyer_speed);
	pros::delay(20);

	drive(-510, 1840, 358, 127, 100, 70, 10, true, false);
	drive(-510, 2240, 0, 127, 100, 70, 10, true, false);

	drive(-440, 2555, 272, 120, 110, 30, 3, true, false);
	drive(-1400, 2570, 272, 110, 40, 1, 3, true, false, true);

	scoreAndStore(2);

	reverseIntakeOneThird();

	drive(-480, 2580, 270, 127, 40, 70, 8, false, false);

	turnOnIntake();

	drive(-330, 2580, 270, 127, 40, 100, 8, false, false, false, true);

	drive(-560, 3775, 307, 127, 127, 270, 5, false, false, false, false, true);
	drive(-820, 3980, 273, 127, 127, 40, 6, false, false, false, false, true);

	pros::delay(20);
	stopHold();

	drive(-700, 4280, 273, 120, 120, 70, 6, false, true);

	drive(-690, 4640, 313, 120, 120, 40, 4, true, false);

	drive(-1120, 5070, 313, 120, 40, 1, 5, true, false, true);

	scoreAndStore(4);

	reverseIntakeOneThird();

	drive(-740, 4705, 316, 120, 127, 70, 5, false, false);

	turnOnIntake();

	drive(-720, 4480, 325, 127, 100, 100, 5, false, false);

	drive(-800, 4450, 330, 127, 127, 150, 5, false, true);

	drive(-780, 4570, 79, 127, 127, 100, 7, false, true);

	drive(-300, 4580, 89, 127, 127, 100, 3, false, true);

	drive(-220, 2815, 91, 120, 100, 100, 2, true, false);

	drive(300, 2570, 91, 85, 85, 20, 2, true, false);

	drive(1500, 2550, 92, 95, 95, 1, 2, true, false, true);

	scoreAndStore(3);

	reverseIntakeHalf();

	drive(810, 2580, 90, 127, 40, 100, 5, false, true);

	turnOnIntake();
	stopHold();
	pros::delay(30);

	drive(900, 3190, 60, 127, 127, 150, 8, false, true);
	drive(1470, 3290, 57, 127, 127, 150, 8, false, true);

	drive(1570, 4300, 1, 127, 100, 70, 6, true, false);
	drive(1570, 4700, 1, 120, 120, 30, 3, true, false);
	drive(1560, 4900, 1, 120, 40, 1, 5, true, false, true);

	scoreAndStore(5);

	reverseIntakeOneThird();

	drive(1540, 4420, 0, 127, 127, 100, 5, false, true);

	turnOnIntake();

	drive(1910, 4200, 65, 127, 127, 100, 5, false, true);

	drive(3280, 4300, 65, 127, 127, 80, 7, true, false);

	drive(3870, 4870, 43, 100, 100, 30, 3, true, false);

	drive(4000, 5000, 43, 120, 40, 1, 3, true, false, true);

	scoreAndStore(4);

	reverseIntakeHalf();

	drive(3790, 4680, 43, 127, 127, 150, 7, false, false, false, true);

	turnOnIntake();

	drive(3690, 4680, 43, 127, 127, 150, 7, false, false, false, true);

	drive(3580, 4100, 90, 127, 127, 100, 5, false, false, false, true);

	drive(3895, 3920, 90, 127, 127, 100, 5, false, false, false, true);

	stopHold();
	pros::delay(10);

	drive(3580, 3235, 175, 127, 127, 100, 5, true, false);
	drive(3580, 2835, 175, 127, 127, 80, 5, true, false);
	drive(3500, 2615, 91, 120, 110, 30, 5, true, false);

	drive(4210, 2580, 91, 120, 40, 1, 5, true, false, true);

	scoreAndStore(2);

	reverseIntakeOneThird();

	drive(3470, 2520, 90, 127, 127, 200, 5, false, true);

	turnOnIntake();

	drive(3420, 2510, 258, 127, 127, 200, 5, false, false, false, true);
	drive(2830, 2500, 269, 127, 127, 200, 5, false, false, false, true);

	drive(2280, 1630, 265, 127, 127, 100, 5, true, false);
	drive(1745, 1630, 265, 127, 127, 80, 5, true, false);

	drive(1610, 400, 182, 100, 100, 30, 3, true, false);

	drive(1610, -300, 180, 120, 40, 1, 5, true, false, true);

	scoreAndStore(2);

	reverseIntakeOneThird();

	drive(1630, 600, 180, 127, 127, 100, 3, false, false);

	turnOnIntake();

	drive(2450, 1070, 137, 127, 127, 100, 6, false, true);
	drive(2910, 750, 137, 127, 127, 100, 6, false, true);

	drive(3550, 945, 90, 127, 127, 100, 6, true, false);
	drive(3875, 930, 90, 120, 120, 70, 6, true, false);
	pros::delay(20);

	drive(3740, 480, 137, 120, 120, 30, 2, true, false);
	drive(4220, 0, 137, 110, 40, 1, 3, true, false, true);

	scoreAndStore(1);

	reverseIntakeOneThird();

	drive(3770, 400, 133, 90, 90, 100, 6, false, true);

	turnOnIntake();

	drive(3720, 450, 133, 90, 90, 100, 6, false, true);

	pros::lcd::set_text(5, std::to_string(pros::millis()-initial_time));
	pros::lcd::set_text(6, "Completed");

	/**/

	stopCoast();
	turnOffIntake();
}


void opcontrol() {
	while (true) {

		// Position tracking stuff
		float curr_encoder_fwd_rev = (right_encoder.get_value() + left_encoder.get_value()) / 2;
		float curr_encoder_left_right = center_encoder.get_value();

		angle_ = inertial.get_heading();
		if(angle_ > 360) angle_ = 0;
		if(angle_ < 0) angle_ = angle_+360;
		pos_x = pos_x - (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * -sin(angle_*PI/180)) + ((curr_encoder_left_right-prev_encoder_left_right) * -cos(angle_*PI/180)));
		pos_y = pos_y + (((curr_encoder_fwd_rev-prev_encoder_fwd_rev) * cos(angle_*PI/180)) - ((curr_encoder_left_right-prev_encoder_left_right) * sin(angle_*PI/180)));
		prev_encoder_fwd_rev = curr_encoder_fwd_rev;
		prev_encoder_left_right = curr_encoder_left_right;

		pros::lcd::set_text(1, std::to_string(pos_x));
		pros::lcd::set_text(2, std::to_string(pos_y));



		/*AWESOME DRIVE*/

		//Getting raw joystick values
    float up_down = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  	float left_right = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		float turnX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		float turnY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		//Getting current angle
		float angle = inertial.get_heading();
		if(angle < 0) angle = 360 + angle;

		//Calculating left joystick angle
		float joystick_angle = atan(left_right/up_down) * 180 / PI;
		if(up_down < 0) joystick_angle = 180 + joystick_angle;
		if(joystick_angle < 0) joystick_angle = 360 + joystick_angle;

		//Calcualating difference in angles
		float difference = angle - joystick_angle;
		if(difference < 0) difference = 360 + difference;
		difference = 360 - difference;

		//Magnitude calculations
		float magnitude = sqrt(pow(up_down, 2.0) + pow(left_right, 2.0));
		if(magnitude > 127) magnitude = 127;

		//Calculating right joystick angle
		float turn_angle = atan(turnX/turnY) * 180 / PI;
		if(turnY < 0) turn_angle = 180 + turn_angle;
		if(turn_angle < 0) turn_angle = 360 + turn_angle;

		//Calculating turn difference in angles
		float turn_difference = turn_angle - angle;
		if(turn_difference < 0) turn_difference = 360 + turn_difference;
		turn_difference = 360 - turn_difference;

		//Turn magnitude calculations
		float turn_magnitude = (sqrt(pow(turnY, 2.0) + pow(turnX, 2.0)))*((turn_difference/360)+0.5);
		if(turn_magnitude > 127) turn_magnitude = 127;

		//Calculating actual turn value based on joystick position
		if(turn_difference < 180){
			turn_magnitude = -turn_magnitude;}
		if(turn_difference > 180) turn_difference = 360 - turn_difference;
		float proportion = sqrt(turn_difference/60);
		turn_magnitude = turn_magnitude*proportion;
		if(isinf(turn_magnitude)) turn_magnitude = 0;
		if(turn_difference < 1 || turn_difference > 359) turn_magnitude = 0;

		//Actual calculated values for motors
		int actual_up_down = cos(difference * PI / 180.0) * magnitude;
		int actual_left_right = -sin(difference * PI / 180.0) * magnitude;
		int actual_turn = turn_magnitude;

		//Applying final values to motors for motion
    left_wheel_front.move(actual_up_down - actual_left_right + actual_turn);
		left_wheel_back.move(actual_up_down + actual_left_right + actual_turn);
    right_wheel_front.move(-actual_up_down - actual_left_right + actual_turn);
		right_wheel_back.move(-actual_up_down + actual_left_right + actual_turn);

		//Intake subsytem controls
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			left_intake.move(intake_speed);
			right_intake.move(-intake_speed);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			left_intake.move(-intake_speed);
			right_intake.move(intake_speed);
		}
		else{
			left_intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			left_intake.move(0);
			right_intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			right_intake.move(0);}

		pros::lcd::set_text(3, std::to_string(ball_limit_switch2.get_value()));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			feeder_middle.move(-conveyer_speed);
			feeder_top.move(-conveyer_speed);
		}
		else{
			//Stops the conveyer and indexer if button is not clicked
			feeder_middle.move(0);
			feeder_top.move(0);
		}

		//Saves resources
    pros::delay(1);
  }
}
