#include <iostream>
#include <cmath>
#include <eigen/Eigen/Dense>
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include "odo_speed_profile.h"
#include "global.h"
#include "cox.h"
#include "Camera.h"

extern "C" {
#include "spi_com.h"
}

using namespace std;
using namespace Eigen;

double radius = 15;
double gearReduction = 6.6;
double gearBoxReduction = 18;
double length = 250;
int encoder = 1024;
int encoderCounter = 4;
double nbrPulsPerTurn = encoder * encoderCounter * gearReduction * gearBoxReduction;
double peri = (2 * M_PI * radius);
double mm_per_pulse = peri / nbrPulsPerTurn;
double wheel_base = 130;
static const int SPI_Channel = 1;
position_s odometry_t1;
position_s correction;
position_s odometry_t;
int odometry_done;
int kalman_count;
int flag_check_for_returning = 0;

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
	kalman_out.x = odo.x - kal(0);
	kalman_out.y = odo.y - kal(1);
	kalman_out.ang = odo.ang - kal(2);
	kalman_out.Cov = C_kalman - odo.Cov;
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
	if(cox.x == 0 && cox.y == 0 && cox.ang == 0)
	{
		kalman_out.x = odometry_t.x;
		kalman_out.y = odometry_t.y;
		kalman_out.ang = odometry_t.ang;
		kalman_out.Cov = odometry_t.Cov;
	}
	else
	{
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
	}	
	return kalman_out;
}

position_s odometry(double wheel1, double wheel2, double dr_old, double dl_old, double x_old, double y_old, double a_old, Matrix3d cxya_old)
{



	MatrixXd Cu(2, 2), Axya(3,3), Au(3,2), cxya(3, 3);
	
	// Load encoder values
	position_s pos;
	double x_new, y_new, a_new, Cv;
	double d_dr;
	double d_dl;
	//cout << "the difference in encoder is : "<< d_dr << " " <<d_dl <<endl;	

	double d_d ;
	double d_a;

	double d_x;
	double d_y;
	
	double sigma_r, sigma_l;
	sigma_r = 0.05;
	sigma_l = 0.05;
	

	d_dr = wheel1 - dr_old;
	d_dl = wheel2 - dl_old;
	//cout << "the difference in encoder is : "<< d_dr << " " <<d_dl <<endl;	

	d_d = (d_dr + d_dl) / 2;
	d_a = (d_dr - d_dl) / wheel_base;

	d_x = d_d * cos(a_old + (d_a / 2));
	d_y = d_d * sin(a_old + (d_a / 2));
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


void *set_speed(void *)
{	
	/*while(1)
	{
		switch
	}*/

    wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
	int counter =0;
	double angular_set, angluar_w;
	double velocity_set;
	int clock_wise_rot;
	double forward_v ;//  # forward velocity
	double angular_v;//  # steering gain
	double vl;
	double vr;
	MatrixXd cxya_old(3,3);
	double angle_change;
	double angle_stop;
	cxya_old << 1, 0, 0, 0, 1, 0, 0, 0, 1 * M_PI / 180;
	double start_x, start_y, end_x, end_y, speed, oldencoder = 0, oldencoder_l = 0, oldencoder_r = 0, start_a, cur_x, cur_y, cur_a;
	cur_x = 1220;
	cur_y = 390;
	cur_a = (double)(M_PI / 2);
	end_x = cur_x;
	end_y = cur_y;
	MotorData.Set_Speed_M1 = 0;			
	MotorData.Set_Speed_M2 = 0;	     	
	Send_Read_Motor_Data(&MotorData);
	oldencoder_l = MotorData.Encoder_M2;
	oldencoder_r = -MotorData.Encoder_M1;
	cxya_old << 1, 0, 0, 0, 1, 0, 0, 0, 1 * M_PI / 180;
	
	while (1)
	{
		 //# Calculating Velocity_Left & Velocity_Right
			
			if(!contour_not_found)
			{
				if (distance_x >0)
				{

				clock_wise_rot = 1;


				}
			   else
				{

				clock_wise_rot = -1;

				}

				
				if (distance_x * clock_wise_rot > 10)
				{
				  
					angular_set = (300*2*radius)/wheel_base;
				if (contour_area>8000)
			   	{
					 forward_v = 1500*15;
			

				}
				
				else
				{
					forward_v = 3000 * radius;
				}
					
				}
				else
				{
					angular_set = 0;
					if (contour_area>8000)
			   {
					 forward_v = 1500*15;
					 //angular_set = 100;

				}
				else if (contour_area>12000)
				{
					cout<<"Break"<<endl;
					break;
				}
				else
				{
					forward_v = 3000 * radius;
				}
				}
				
			
				
				vl = ( 2 *forward_v + (wheel_base * angular_set * clock_wise_rot))/ (2 * radius);
				vr = (2 * forward_v - (wheel_base * angular_set * clock_wise_rot))/ (2* radius);
			
			}
			else
			{
				vr = -500;
				vl = 500;
				
			}
			
			//counter ++;
			MotorData.Set_Speed_M1 = -vr;			
			MotorData.Set_Speed_M2 = vl;	     	
			Send_Read_Motor_Data(&MotorData);
			
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
				//cur_x = correction.x + odometry_t.x;
				//cur_y = correction.y + odometry_t.y;
				//cur_a = correction.ang + odometry_t.ang;
				cur_x = correction.x ;
				cur_y = correction.y ;
				cur_a = correction.ang ;	
				cxya_old = correction.Cov;
				kalman_count = 1;
				
			}
			
			odometry_t = odometry(-MotorData.Encoder_M1 * mm_per_pulse, MotorData.Encoder_M2 * mm_per_pulse, oldencoder_r * mm_per_pulse, oldencoder_l * mm_per_pulse, cur_x, cur_y, cur_a, cxya_old);
			//cout << "Motor 1" << MotorData.Encoder_M1 << " Motor 2" << MotorData.Encoder_M2 <<endl;
			oldencoder_l = MotorData.Encoder_M2;
			oldencoder_r = -MotorData.Encoder_M1;
			cur_x = odometry_t.x;
			cur_y = odometry_t.y;
			cur_a = odometry_t.ang;
			cxya_old = odometry_t.Cov;
			//cout << "THREAD odo is :" << cur_x << " " <<cur_y << " " << cur_a <<endl;
			//cout << " x, y :" << cur_x << " " << cur_y << endl;
			odometry_done = 1;
			if(scan_match_done == 0) kalman_count = 0;	
			if(collected == 1) 
			{
				angle_stop = cur_a;
				MotorData.Set_Speed_M1 = 0;			
				MotorData.Set_Speed_M2 = 0;	     	
				Send_Read_Motor_Data(&MotorData);				
				break;
				
			}
			//rotation1(3.14);
		}
		cout << " x, y :" << cur_x << " " << cur_y << "cur angle is : " << cur_a << endl;
		
		// code to rotate the bot in the direction of home
	while(1)
	{
		if (angle_stop >= M_PI / 2)
		{
			clock_wise_rot = -1;
			angle_change = (M_PI / 2) + (M_PI - cur_a);
			//cout << "angle change : " << angle_change *  180 / M_PI << endl;
			//cout << " Angle is :" << cur_a * 180 /M_PI <<endl;
			
			//cout <<"scan_match_done : " <<scan_match_done << "Kalman_count :" << kalman_count <<endl;
			if(scan_match_done == 1 && kalman_count == 0)
			{
				
				//cout <<"Scan match done while rotating" <<endl;
				odometry_t1.x = odo_x;
				odometry_t1.y = odo_y;
				odometry_t1.ang = odo_a;
				odometry_t1.Cov = covariance;
				correction = kalman_filter(odometry_t, cox_output_t);
				//old_pos.x = correction.x;
				//old_pos.y = correction.y;
				//old_pos.ang = correction.ang;
				//cur_x = correction.x + odometry_t.x;
				//cur_y = correction.y + odometry_t.y;
				//cur_a = correction.ang + odometry_t.ang;
				cur_x = correction.x ;
				cur_y = correction.y ;
				cur_a = correction.ang ;	
				cxya_old = correction.Cov;
				kalman_count = 1;
				
			}
			
			//vr = 500;
			//vl = -500;
			MotorData.Set_Speed_M1 = -500;			
			MotorData.Set_Speed_M2 = -500;	     	
			Send_Read_Motor_Data(&MotorData);
			odometry_t = odometry(-MotorData.Encoder_M1 * mm_per_pulse, MotorData.Encoder_M2 * mm_per_pulse, oldencoder_r * mm_per_pulse, oldencoder_l * mm_per_pulse, cur_x, cur_y, cur_a, cxya_old);
	//cout << "Motor 1" << MotorData.Encoder_M1 << " Motor 2" << MotorData.Encoder_M2 <<endl;
			oldencoder_l = MotorData.Encoder_M2;
			oldencoder_r = -MotorData.Encoder_M1;
			cur_x = odometry_t.x;
			cur_y = odometry_t.y;
			cur_a = odometry_t.ang;
			cxya_old = odometry_t.Cov;
			if(scan_match_done == 0) kalman_count = 0;
			if( cur_a > (M_PI * 3 / 2)) break;
			
		}
		else
		{
			clock_wise_rot = 1;
			angle_change = (cur_a + (M_PI / 2));
			//cout << "angle change : " << angle_change * 180 / M_PI << endl;
			//cout << " Angle is :" << cur_a * 180 /M_PI <<endl;
			//cout <<"scan_match_done : " <<scan_match_done << "Kalman_count :" << kalman_count <<endl;
			if(scan_match_done == 1 && kalman_count == 0)
			{
				//cout <<"Scan match done while rotating" <<endl;
						
				odometry_t1.x = odo_x;
				odometry_t1.y = odo_y;
				odometry_t1.ang = odo_a;
				odometry_t1.Cov = covariance;
				correction = kalman_filter(odometry_t, cox_output_t);
				//old_pos.x = correction.x;
				//old_pos.y = correction.y;
				//old_pos.ang = correction.ang;
				//cur_x = correction.x + odometry_t.x;
				//cur_y = correction.y + odometry_t.y;
				//cur_a = correction.ang + odometry_t.ang;
				cur_x = correction.x ;
				cur_y = correction.y ;
				cur_a = correction.ang ;	
				cxya_old = correction.Cov;
				kalman_count = 1;
				
			}
			//vr = -500;
			//vl = 500;
			MotorData.Set_Speed_M1 = 500;			
			MotorData.Set_Speed_M2 = 500;	     	
			Send_Read_Motor_Data(&MotorData);
			odometry_t = odometry(-MotorData.Encoder_M1 * mm_per_pulse, MotorData.Encoder_M2 * mm_per_pulse, oldencoder_r * mm_per_pulse, oldencoder_l * mm_per_pulse, cur_x, cur_y, cur_a, cxya_old);
	//cout << "Motor 1" << MotorData.Encoder_M1 << " Motor 2" << MotorData.Encoder_M2 <<endl;
			oldencoder_l = MotorData.Encoder_M2;
			oldencoder_r = -MotorData.Encoder_M1;
			cur_x = odometry_t.x;
			cur_y = odometry_t.y;
			cur_a = odometry_t.ang;
			cxya_old = odometry_t.Cov;
			if(scan_match_done == 0) kalman_count = 0;
			if( cur_a < (-M_PI / 2)) break;
			
		}	
	}	
	cout << " x, y :" << cur_x << " " << cur_y << "cur angle is : " << cur_a << endl;
	while(1)
	{
		if(cur_y - end_y > 1000)
		{
			angular_set = 0;
			forward_v = 3000 * radius;
		}
		else if(cur_x < end_x - 100)
		//if(cur_x < end_x - 20)
		{
			clock_wise_rot = -1;
			angular_set = (300*2*radius)/wheel_base;
			if (cur_y - end_y > 300)
			{
				forward_v = 3000 * radius;
			}
			else if(cur_y - end_y  < 10 )
			{	
				forward_v = 0;
				if(cur_x - end_x  < 10)
				{
					
					angular_set = 0;
					cout << " x, y :" << cur_x << " " << cur_y << endl;
				}
				else
				{
					angular_set = (300*2*radius)/wheel_base;
				}		
			}
			else 
			{
				forward_v = 1500 * radius;
			}
						
		}
		else if(cur_x > end_x + 100)
		//else if(cur_x > end_x + 20)
		{
			clock_wise_rot = 1;
			angular_set = (300*2*radius)/wheel_base;
			if (cur_y - end_y > 300)
			{
				forward_v = 3000 * radius;
				//cout << " cwx, cwy :" << cur_x << " " << cur_y << endl;
				
			}
			else if(cur_y - end_y  < 10 )
			{
				forward_v = 0;
				if(cur_x - end_x  < 10)
				{
					
					angular_set = 0;
					cout << " x, y :" << cur_x << " " << cur_y << endl;
				}
				else
				{
					angular_set = (300*2*radius)/wheel_base;
				}			
			}
			else
			{
				forward_v = 1500 * radius;
			}
			
		}
		else
		{
			angular_set = 0;
			if (cur_y - end_y > 300)
			{
				forward_v = 3000 * radius;
				//cout << " acwx, acwy :" << cur_x << " " << cur_y << endl;
			}
			else if(cur_y - end_y  < 10 )
			{
				forward_v = 0;
				if(cur_x - end_x  < 10)
				{
					
					angular_set = 0;
					cout << " x, y :" << cur_x << " " << cur_y << endl;
				}
				else
				{
					angular_set = (300*2*radius)/wheel_base;
				}		
			}
			else
			{
				forward_v = 1500 * radius;
			}
			
		}
		vl = ( 2 *forward_v + (wheel_base * angular_set * clock_wise_rot))/ (2 * radius);
		vr = (2 * forward_v - (wheel_base * angular_set * clock_wise_rot))/ (2* radius);
		MotorData.Set_Speed_M1 = -vr;			
		MotorData.Set_Speed_M2 = vl;	     	
		Send_Read_Motor_Data(&MotorData);
		//cout <<"scan_match_done : " <<scan_match_done << "Kalman_count :" << kalman_count <<endl;
		if(scan_match_done == 1 && kalman_count == 0)
			{
				
				//cout <<"Scan match done while returning home" <<endl;
				odometry_t1.x = odo_x;
				odometry_t1.y = odo_y;
				odometry_t1.ang = odo_a;
				odometry_t1.Cov = covariance;
				correction = kalman_filter(odometry_t, cox_output_t);
				//old_pos.x = correction.x;
				//old_pos.y = correction.y;
				//old_pos.ang = correction.ang;
				//cur_x = correction.x + odometry_t.x;
				//cur_y = correction.y + odometry_t.y;
				//cur_a = correction.ang + odometry_t.ang;
				cur_x = correction.x ;
				cur_y = correction.y ;
				cur_a = correction.ang ;	
				cxya_old = correction.Cov;
				kalman_count = 1;
				
			}
			
			odometry_t = odometry(-MotorData.Encoder_M1 * mm_per_pulse, MotorData.Encoder_M2 * mm_per_pulse, oldencoder_r * mm_per_pulse, oldencoder_l * mm_per_pulse, cur_x, cur_y, cur_a, cxya_old);
			//cout << "Motor 1" << MotorData.Encoder_M1 << " Motor 2" << MotorData.Encoder_M2 <<endl;
			oldencoder_l = MotorData.Encoder_M2;
			oldencoder_r = -MotorData.Encoder_M1;
			cur_x = odometry_t.x;
			cur_y = odometry_t.y;
			cur_a = odometry_t.ang;
			cxya_old = odometry_t.Cov;
			//cout << "THREAD odo is :" << cur_x << " " <<cur_y << " " << cur_a <<endl;
			//cout << " x, y :" << cur_x << " " << cur_y << endl;
			odometry_done = 1;
			if(scan_match_done == 0) kalman_count = 0;
	}	
	cout << " x, y :" << cur_x << " " << cur_y << endl;	
		
}
