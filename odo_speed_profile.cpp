#include <iostream>
#include <cmath>
#include <eigen/Eigen/Dense>
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include "odo_speed_profile.h"
#include "global.h"
#include "cox.h"

extern "C" {
#include "spi_com.h"
}

using namespace std;
using namespace Eigen;

position_s odometry_t;

int odometry_done = 0;
float radius = 15;
float gearReduction = 6.6;
float gearBoxReduction = 18;
float length = 250;
int encoder = 1024;
int encoderCounter = 4;
float nbrPulsPerTurn = encoder * encoderCounter * gearReduction * gearBoxReduction;
float peri = (2 * M_PI * radius);

float mm_per_pulse = peri / nbrPulsPerTurn;
float wheel_base = 130;
static const int SPI_Channel = 1;
short angle_rot; // this decides if there is a need to turn the robot before going in a straight line, set by getangularspeedprofile function
float angle_change;
float angle_diff_final;
int kalman_count;
position_s correction;


/*
position_s kalman_filter(position_s odo, position_s cox) {
	position_s kalman_out;
	double angle_diff = cox.ang - odo.ang;
	angle_diff = findmod(angle_diff + M_PI, 2 * M_PI) - M_PI;
	MatrixXd X1(1, 3), X2(1, 3), kal1(1, 3), kal2(1, 3), kal(1, 3);
	Matrix3d C_kalman;
	X1 << odo.x, odo.y, angle_diff;
	X2 << cox.x, cox.y, 0;
	kal1 = (cox.Cov * (cox.Cov + odo.Cov).inverse()) * (X1.transpose());
	kal2 = (odo.Cov * (cox.Cov + odo.Cov).inverse()) * (X2.transpose());
	C_kalman = (cox.Cov.inverse() + odo.Cov.inverse()).inverse();
	kal = (kal1 + kal2);
	
	/*kalman_out.x = odo.x - kal(0);
	kalman_out.y = odo.y - kal(1);
	kalman_out.ang = odo.ang - kal(2);*/
	
	/*kalman_out.x = kal(0);
	kalman_out.y = kal(1);
	kalman_out.ang = kal(2);
	kalman_out.Cov =  C_kalman;

	
	//cout << "angle " << odo.ang << "  " << cox.ang << " " << kal(2) << endl;
	//cout << "x " << odo.x << "  " << cox.x << " " << kal(0) << endl;
	//cout << "y " << odo.y << "  " << cox.y << " " << kal(1) << endl;
	
	return kalman_out;
}*/

position_s kalman_filter(position_s odo, position_s cox) {
	position_s kalman_out;
	//double angle_diff = cox.ang - odo.ang;
	//angle_diff = findmod(angle_diff + M_PI, 2 * M_PI) - M_PI;
	MatrixXd X1(1, 3), X2(1, 3), kal1(1, 3), kal2(1, 3), kal(1, 3);
	Matrix3d C_kalman;
	X1 << odo.x, odo.y, odo.ang;
	X2 << cox.x, cox.y, cox.ang;
	kal1 = (cox.Cov * (cox.Cov + odo.Cov).inverse()) * (X1.transpose());
	kal2 = (odo.Cov * (cox.Cov + odo.Cov).inverse()) * (X2.transpose());
	C_kalman = (cox.Cov.inverse() + odo.Cov.inverse()).inverse();
	kal = (kal1 + kal2);
	//kalman_out.x =  odo.x - kal(0);
	//kalman_out.y =  odo.y - kal(1) ;
	//kalman_out.ang = odo.ang - kal(2);
	kalman_out.x = kal(0);
	kalman_out.y = kal(1);
	kalman_out.ang = kal(2);
	kalman_out.Cov = C_kalman;
	//cout << "angle " << odo.ang << "  " << cox.ang << " " << kal(2) << endl;
	//cout << "x " << odo.x << "  " << cox.x << " " << kal(0) << endl;
	//cout << "y " << odo.y << "  " << cox.y << " " << kal(1) << endl;
	
	return kalman_out;
}

motor_speed_s inverse_kinematic(speed_s feedback)
{
	motor_speed_s sp;
	sp.v_r = ((2 * feedback.velocity) + (feedback.angular_velocity * wheel_base)) / (2 * radius);
	sp.v_l = ((2 * feedback.velocity) - (feedback.angular_velocity * wheel_base)) / (2 * radius);
	sp.v_r *= (1 / mm_per_pulse);
	sp.v_l *= (1 / mm_per_pulse);
	return sp;
}

VectorXd  get_speed_profile(float start_x, float start_y, float end_x, float end_y)
{
	float startPos = 0;
	float maxAcc = 1000; // Acceleration
	float deAcc = -1000; // Deacceleration
	float maxSpeed = 3000; // Max Speed

	float h = 0.1; // SampleTime

	float dist = sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));

	float endPos = (nbrPulsPerTurn * (dist * 1.26)) / peri;
	//float endPos = sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));
	//float endPos = dist;
	// Initialization
	float breakPos = 0;
	float position = startPos;
	float speed = 0;
	float oldSpeed = 0;
	int n = 0;

	VectorXd pos(1000000), sp(1000000), acc(1000000);
	// Program
	if (endPos > startPos) {
		while (endPos > position && speed >= 0) {

			// Calculation
			if (speed < maxSpeed && position < endPos + breakPos) {
				speed = oldSpeed + maxAcc * h;
				breakPos = speed * speed / (2 * deAcc);

			}
			else if (position > endPos + breakPos) {

				speed = oldSpeed + (deAcc * h);
			}
			else {

				speed = maxSpeed;
			}
			position = position + speed * h;

			// save variables for plotting
			pos(n) = position;
			sp(n) = speed;
			acc(n) = (speed - oldSpeed) / h;

			// update oldSpeed
			oldSpeed = speed;
			n++;
		}
	}
	//else {
	/*endPos = 0;
	speed = 0;
	while (position > endPos && speed <= 0) {
		// Calculation
		if (speed > -maxSpeed && position > endPos + breakPos) {
			speed = oldSpeed - maxAcc * h;
			breakPos = -speed * speed / (2 * deAcc);
		}
		else if (position < endPos + breakPos) {
			speed = oldSpeed - deAcc * h;
		}
		else {
			speed = -maxSpeed;
		}
		position = position + speed * h;

		// save variables for plotting
		pos(n) = position;
		sp(n) = speed;
		acc(n) = (speed - oldSpeed) / h;

		// update oldSpeed
		oldSpeed = speed;
		n++;
	}*/


	VectorXd position_v = pos.head(n);
	VectorXd velocity = sp.head(n);
	//position_v = position_v * ();
	return position_v;
}



short Des_Speed = -500;
short Des_Speed1 = -500;
int Select = 0;
int Counter = 0;

speed_s position_feedback_controller(position_s pos, position_s oldpos)
{
	speed_s return_s;
	float t = 0.1;

	return_s.velocity = sqrt(pow((pos.x - oldpos.x), 2) + pow((pos.y - oldpos.y), 2)) / t;
	return_s.angular_velocity = (pos.ang - oldpos.ang) / t;
	return return_s;
}

/*
The odometry used for speed profile 
*/
position_s odometry_only(float wheel1, float wheel2, float dr_old, float dl_old, float x_old, float y_old, float a_old)
{
	     
	     
	
	     // Load encoder values
	    position_s pos;
	    float x_new, y_new, a_new;
	      
	    
	    double d_dr = wheel1 - dr_old;
            double d_dl = wheel2 - dl_old;
	    //cout << "the difference in encoder is : "<< d_dr << " " <<d_dl <<endl;	

 	    double d_d = (d_dr + d_dl) / 2;
            double d_a = (d_dr - d_dl) / wheel_base;
	    
	    double d_x = d_d * cos(a_old + (d_a / 2));
            double d_y = d_d * sin(a_old + (d_a / 2));
	    a_new = findmod(a_old + d_a, 2 * M_PI);
	    x_new = d_x + x_old;
            y_new = d_y + y_old;
      
	    pos.x = x_new;
	    pos.y = y_new;
            pos.ang = a_new;
	    return pos;	
}


/*

 wheel1: Encoder value for left wheel
 wheel2 : Encoder value for right wheel
 dr_old : Previous right wheel encoder value
 dl_old : Previous left wheel encoder value
 x_old  : previous x position
 y_old  : previous y position
 a_old  : previous angle
*/

position_s odometry(float wheel1, float wheel2, float dr_old, float dl_old, float x_old, float y_old, float a_old, Matrix3d cxya_old)
{



	MatrixXd Cu(2, 2), Axya(3,3), Au(3,2), cxya(3, 3);
	
	// Load encoder values
	position_s pos;
	float x_new, y_new, a_new, Cv;
	
	float sigma_r, sigma_l;
	//sigma_r = 0.05;
	//sigma_l = 0.05;
	

	double d_dr = wheel1 - dr_old;
	double d_dl = wheel2 - dl_old;
	//cout << "the difference in encoder is : "<< d_dr << " " <<d_dl <<endl;	
	sigma_r = fabs(d_dr) * 0.1;
	sigma_l = fabs(d_dr) * 0.1;
	double d_d = (d_dr + d_dl) / 2;
	double d_a = (d_dr - d_dl) / wheel_base;

	double d_x = d_d * cos(a_old + (d_a / 2));
	double d_y = d_d * sin(a_old + (d_a / 2));
	a_new = findmod(a_old + d_a, 2 * M_PI);
	x_new = d_x + x_old;
	y_new = d_y + y_old;
	Axya << 1, 0, -d_y, 0, 1, d_x, 0, 0, 1;
	Au << cos(a_old + (d_a / 2)), -d_d * 0.5 * sin(a_old + (d_a / 2)),
		sin(a_old + (d_a / 2)), d_d * 0.5 * cos(a_old + (d_a / 2)),
		0, 1;
	Cv = ((sigma_r * sigma_r) - (sigma_l * sigma_l)) / (2 * wheel_base);
	Cu << ((sigma_r * sigma_r) + (sigma_l * sigma_l)) / 4, Cv,
		Cv, ((sigma_r * sigma_r) + (sigma_l * sigma_l)) / (wheel_base * wheel_base);
	cxya = Axya * cxya_old * Axya.transpose() + Au * Cu * Au.transpose();


	pos.x = x_new;
	pos.y = y_new;
	pos.ang = a_new;
	pos.Cov = cxya;
	return pos;
}



VectorXd getAnglularSpeedProfile(float start_x, float start_y, float end_x, float end_y, float oldangle)
{
	float y, x, angle;
	y = end_y - start_y;
	x = end_x - start_x;
	float endPos, startPos, position, breakPos, speed = 0, oldSpeed, h = 0.1;
	float maxAcc = 50; // Acceleration
	float deAcc = -50; // Deacceleration
	float maxSpeed = 1000; // Max Speed
	int n = 0;
	VectorXd pos(1000000), sp(1000000), acc(1000000);

	if (y == 0)
	{
		angle = -90 * M_PI / 180;
	}
	else
	{
		angle = atan2(-x, y);
	}

	cout << "angle is :" << angle * 180 / M_PI << endl;
	if (angle == 0)
	{
		angle_rot = 0;
	}
	else if (angle > 0)
	{
		angle_rot = 1;
	}
	else
	{
		angle_rot = -1;
	}
	cout << " the angle is : " << angle << endl;

	endPos = (nbrPulsPerTurn * angle * angle_rot * length) / peri;
	//cout << " The endpos is : " << endPos << endl;
	if (angle_rot != 0) {
		while (endPos > position && speed >= 0) {
			// Calculation
			if (speed < maxSpeed && position < endPos + breakPos) {
				speed = oldSpeed + maxAcc * h;
				breakPos = speed * speed / (2 * deAcc);

			}
			else if (position > endPos + breakPos) {

				speed = oldSpeed + (deAcc * h);
			}
			else {

				speed = maxSpeed;
			}
			position = position + speed * h;

			// save variables for plotting
			pos(n) = position;
			sp(n) = speed;
			acc(n) = (speed - oldSpeed) / h;

			// update oldSpeed
			oldSpeed = speed;
			n++;
			//cout << speed <<endl;
		}
	}
	angle_change = angle;
	VectorXd angProf = pos.head(n);

	return angProf;

}

VectorXd end_angle(float angle1)
{
	float angle;


	float endPos, startPos, position, breakPos, speed = 0, oldSpeed, h = 0.1;
	float maxAcc = 50; // Acceleration
	float deAcc = -50; // Deacceleration
	float maxSpeed = 1000; // Max Speed
	int n = 0;
	VectorXd pos(1000000), sp(1000000), acc(1000000);


	angle = angle1;
	if(angle_change>0)
	{

		angle_diff_final=angle_change-(2*M_PI+angle1);
	}
	else if(angle_change<0)
	{
		if(angle1>angle_change)
		{		
			angle_diff_final=angle+angle_change;
		}
		else	
		{
			angle_diff_final=angle-angle_change;
		}
	}


	/*cout << "angle is :" << angle * 180 / M_PI << endl;
	if (angle == 0)
	{
		angle_rot = 0;
	}
	else if (angle > 0)
	{
		angle_rot = 1;
	}
	else
	{
		angle_rot = -1;
	}*/
	cout << " the angle is : " << angle << endl;

	endPos = (nbrPulsPerTurn * angle * angle_rot * length) / peri;
	//cout << " The endpos is : " << endPos << endl;
	if (angle_rot != 0) {
		while (endPos > position && speed >= 0) {
			// Calculation
			if (speed < maxSpeed && position < endPos + breakPos) {
				speed = oldSpeed + maxAcc * h;
				breakPos = speed * speed / (2 * deAcc);

			}
			else if (position > endPos + breakPos) {

				speed = oldSpeed + (deAcc * h);
			}
			else {

				speed = maxSpeed;
			}
			position = position + speed * h;

			// save variables for plotting
			pos(n) = position;
			sp(n) = speed;
			acc(n) = (speed - oldSpeed) / h;

			// update oldSpeed
			oldSpeed = speed;
			n++;
			//cout << speed <<endl;
		}
	}
	angle_change = angle;
	VectorXd angProf = pos.head(n);

	return angProf;

}


// Motor m1 is right wheel motor and m2 is left wheel
// Motor M1 should be given -value to move it forward. 
void * set_speed(void *) {

	position_s odometry_t1;
        
	wiringPiSetup();
	wiringPiSPISetup(SPI_Channel, 1000000);
	float start_x, start_y, end_x, end_y, speed, oldencoder = 0, oldencoder_l = 0, oldencoder_r = 0, start_a, cur_x, cur_y, cur_a;
	start_x = 500;
	start_y = 600;
	end_x = 500;
	end_y = 1600;
	start_a = 90 * M_PI / 180;
	int i, n;
	double velocity;
	//float last_ang = (45) * M_PI / 180;
	VectorXd encoder_ang = getAnglularSpeedProfile(start_x, start_y, end_x, end_y, start_a);
	//cout << " exited from angular speed profile \n";
	cur_x = start_x;
	cur_y = start_y;
	cur_a = start_a;
	n = encoder_ang.size();



	if (angle_rot != 0)
	{
		n = encoder_ang.size();
		for (i = 1; i < n; i++)
		{
			speed = (encoder_ang(i) - encoder_ang(i - 1)) / 0.1;

			if (angle_rot > 0)
			{
				MotorData.Set_Speed_M1 = -speed / 2;
				MotorData.Set_Speed_M2 = -speed / 2;

			}
			else
			{
				MotorData.Set_Speed_M2 = speed / 2;
				MotorData.Set_Speed_M1 = speed / 2;
			}

			Send_Read_Motor_Data(&MotorData);
			oldencoder = encoder_ang(i);

		}

	}
	start_a = start_a + angle_change;
	VectorXd encoder_val = get_speed_profile(start_x, start_y, end_x, end_y);
	encoder_val *= mm_per_pulse;
	VectorXd pos_v(3);
	//cout << encoder_val <<endl;
	n = encoder_val.size();
	position_s pos_speed_prof_t[n];
	
	oldencoder = 0;
	cur_x = start_x;
	cur_y = start_y;
	cur_a = start_a;
	pos_speed_prof_t[0].x = cur_x;
	pos_speed_prof_t[0].y = cur_y;
	pos_speed_prof_t[0].ang = cur_a;
	MatrixXd cxya_old(3,3);
	cxya_old << 1, 0, 0, 0, 1, 0, 0, 0, 1 * M_PI / 180;
	cout << "encoder value of speed profile : " <<encoder_val(n -1) <<endl;
	for (i = 1; i < n; i++)
	{

		pos_speed_prof_t[i] = odometry_only(encoder_val(i), encoder_val(i), encoder_val(i - 1), encoder_val(i - 1), cur_x, cur_y, cur_a);
		cur_x = pos_speed_prof_t[i].x;
		cur_y = pos_speed_prof_t[i].y;
		cur_a = pos_speed_prof_t[i].ang;
	}
	cur_x = start_x;
	cur_y = start_y;
	cur_a = start_a;
	position_s old_pos;
	old_pos.x = cur_x;
	old_pos.y = cur_y;
	old_pos.ang = cur_a;
	speed_s speed_t;

	MotorData.Set_Speed_M1 = 0;
	MotorData.Set_Speed_M2 = 0;
	Send_Read_Motor_Data(&MotorData);

	oldencoder_l = MotorData.Encoder_M2;
	oldencoder_r = -MotorData.Encoder_M1;


	i = 0;
	motor_speed_s motor_speed_val_t;
	
	float delta_x, delta_y;
	cout << "just before while\n";
	for (i = 0; i < n; i ++)
	{
		delta_x = end_x - cur_x;
		delta_y = end_y - cur_y;
		if(fabs(delta_x) <= 70 && fabs(delta_y) <= 70)
		{
			MotorData.Set_Speed_M1 = 0;
			MotorData.Set_Speed_M1 = 0;
			Send_Read_Motor_Data(&MotorData);
			break;
		}
			
		
		
		if(scan_match_done == 1 && kalman_count == 0)
		{
			
			
			odometry_t1.x = odo_x;
			odometry_t1.y = odo_y;
			odometry_t1.ang = odo_a;
			odometry_t1.Cov = covariance;
			correction = kalman_filter(odometry_t, cox_output_t);
			//old_pos.x = correction.x;
			//old_pos.y = correction.y;
			//old_pos.ang = correction.ang;
			// cur_x = correction.x + odometry_t.x;
			// cur_y = correction.y + odometry_t.y;
			// cur_a = correction.ang + odometry_t.ang;
			cur_x = correction.x ;
			cur_y = correction.y ;
			cur_a = correction.ang ;			
			cxya_old = correction.Cov;
			kalman_count = 1;
		}
		
		speed_t = position_feedback_controller(pos_speed_prof_t[i], old_pos);
		old_pos = pos_speed_prof_t[i];
		//motor_speed_val_t = inverse_kinematic(speed_t);
		MotorData.Set_Speed_M1 = -speed_t.velocity / mm_per_pulse;
		MotorData.Set_Speed_M2 = speed_t.velocity / mm_per_pulse;
		Send_Read_Motor_Data(&MotorData);
		


		odometry_t = odometry(-MotorData.Encoder_M1 * mm_per_pulse, MotorData.Encoder_M2 * mm_per_pulse, oldencoder_r * mm_per_pulse, oldencoder_l * mm_per_pulse, cur_x, cur_y, cur_a, cxya_old);
		//cout << "Motor 1" << MotorData.Encoder_M1 << " Motor 2" << MotorData.Encoder_M2 <<endl;
		oldencoder_l = MotorData.Encoder_M2;
		oldencoder_r = -MotorData.Encoder_M1;
		cur_x = odometry_t.x;
		cur_y = odometry_t.y;
		cur_a = odometry_t.ang;
		cxya_old = odometry_t.Cov;
		//cout << "THREAD odo is :" << cur_x << " " <<cur_y << " " << cur_a <<endl;
		odometry_done = 1;
		if(scan_match_done == 0) kalman_count = 0;

	}


	
	//angle_diff_final = findmod(angle_diff_final + M_PI, 2 * M_PI) - M_PI;


	/*VectorXd last_enc_angle = end_angle(-30);
	if (angle_rot != 0)
	{
		n = encoder_ang.size();
		for (i = 1; i < n; i++)
		{
			speed = (encoder_ang(i) - encoder_ang(i - 1)) / 0.1;

			if (angle_rot > 0)
			{
				MotorData.Set_Speed_M1 = -speed / 2;
				MotorData.Set_Speed_M2 = -speed / 2;

			}
			else
			{
				MotorData.Set_Speed_M2 = speed / 2;
				MotorData.Set_Speed_M1 = speed / 2;
			}

			Send_Read_Motor_Data(&MotorData);
			oldencoder = encoder_ang(i);

		}

	}*/
	//float angle_diff_final = (135*M_PI/180) - angle_change;
	//angle_diff_final = findmod(angle_diff_final + M_PI, 2 * M_PI) - M_PI;
	
		


	/*last_enc_angle = end_angle(angle_diff_final);
	if (angle_rot != 0)
	{
		n = encoder_ang.size();
		for (i = 1; i < n; i++)
		{
			speed = (encoder_ang(i) - encoder_ang(i - 1)) / 0.1;

			if (angle_rot > 0)
			{
				MotorData.Set_Speed_M1 = -speed / 2;
				MotorData.Set_Speed_M2 = -speed / 2;

			}
			else
			{
				MotorData.Set_Speed_M2 = speed / 2;
				MotorData.Set_Speed_M1 = speed / 2;
			}

			Send_Read_Motor_Data(&MotorData);
			oldencoder = encoder_ang(i);

		}

	}*/

	
	/*VectorXd last_enc_angle = end_angle(last_ang);
	if (angle_rot != 0)
	{
		n = encoder_ang.size();
		for (i = 1; i < n; i++)
		{
			speed = (encoder_ang(i) - encoder_ang(i - 1)) / 0.1;

			if (angle_rot > 0)
			{
				MotorData.Set_Speed_M1 = -speed / 2;
				MotorData.Set_Speed_M2 = -speed / 2;

			}
			else
			{
				MotorData.Set_Speed_M2 = speed / 2;
				MotorData.Set_Speed_M1 = speed / 2;
			}

			Send_Read_Motor_Data(&MotorData);
			oldencoder = encoder_ang(i);

		}

	}*/
	

}
