#include <eigen/Eigen/Dense>
#include <iostream>
#include <cmath>
#include "lidar_server.h"
#include "cox.h"
#include <unistd.h>
#include "odo_speed_profile.h"
#include "global.h"








using namespace std;
using namespace Eigen;

Matrix3d covariance;
double odo_x;
double odo_y;
double odo_a;

int scan_match_done = 0;
position_s cox_output_t;



bool isDoubleNaN(double number)
{
return std::isnan(number);
}




double findmod(double x, double y)
{
    double mod;
    if (x < 0) mod = -x;
    else mod = x;
    if (y < 0) y = -y;
    while (mod >= y)
    {
        mod = mod - y;
    }
    if (x < 0) return -mod;
    return mod;


}
double median(VectorXd Vector, int size) {
    sort(Vector.data(), Vector.data() + Vector.size());
    if (size % 2 != 0)
        return (double)Vector[size / 2];
    return (double)(Vector[(size - 1) / 2] + Vector[size / 2]) / 2.0;
}
void *scan_match(void *)
{


	MatrixXd ref(4, 2), lines(4, 2), line_model(4, 4);
	ref << 0, 0,
		2420, 0,
		2420, 3635,
		0, 3630;

	lines << 1, 2,
		2, 3,
		3, 4,
		4, 1;

	line_model << 0, 0, 2430, 0,
		2430, 0, 2430, 3630,
		2430, 3630, 0, 3635,
		0, 3635, 0, 0;

	int No_Meas = 500;
	float pi = 3.1416;
	cout << "inside scan match" << endl;

	MatrixXd Z(2, 2), T(2, 2), z2(2, 2), rot_v(2, 2), z(2, 2), dif(2, 2);
	MatrixXd lr(1, 2), lu(1, 2);
	MatrixXd x(1, No_Meas), y(1, No_Meas), a(1, No_Meas), bias1(1, No_Meas);
	bias1 = MatrixXd::Ones(1, No_Meas);
	rot_v << 0, -1,
		1, 0;
	double rx, ry, ra, rx_init, ry_init, ra_init, unit_v_x, unit_v_y;
	double r, dx, dy, da;
	double alpha, gamma, beta;

	alpha = 0;
	beta = 0;
	gamma = 0 * (pi / 180);

	rx_init = 1400; //1210;
	ry_init = 1200;//380;
	ra_init = 90 * (pi / 180);

	odo_x = 500;
	odo_y = 600;
	odo_a = 90 * M_PI / 180;
	int max_iterations = 15, i, j;
	VectorXd b(3);
	b << 0, 0, 0;
	MatrixXd ui(4, 2), ui_new(No_Meas, 2);
	MatrixXd ri(4, 5);
	MatrixXd midpi(4, 2);
	MatrixXd line_matrix(4, 7);
	MatrixXd Sens_to_robo_rot(3, 3), robo_to_world_rot(3, 3),
		robo_cord(3, No_Meas), robo_cord1(3, No_Meas), robo_cord_rc(3, No_Meas);
	MatrixXd world_cord(No_Meas, 3), ind_dis_abs(No_Meas, 3), world_cord_updated(No_Meas, 6);
	VectorXd Yi;
	bool condition;
	
while(1){
    
		if (Array_full_flag == 0){
			scan_match_done = 0;
		}
				
		//cout << "scan_match_done: " << scan_match_done << "Array_full_flag" << Array_full_flag <<"kalman_count: " << kalman_count <<endl;
		if(scan_match_done == 0 && Array_full_flag == 1 && kalman_count == 0 )
		{
                        /*rx_init needs to be modified later */


		odo_x = odometry_t.x;
		odo_y = odometry_t.y;
		odo_a = odometry_t.ang;
		covariance = odometry_t.Cov; 
		//covariance << 100, 100, 100, 100, 100, 100, 100, 100, 100;



		rx = odo_x;
		ry = odo_x;
		ra = odo_a;
		//cout << "odo values in cox " << odo_x << "  " <<odometry_t.x << endl;
		ang = -ang;

		midpi << (line_model(0, 0) + line_model(0, 2)) / 2,
			(line_model(0, 1) + line_model(0, 3)) / 2,
			(line_model(1, 0) + line_model(1, 2)) / 2,
			(line_model(1, 1) + line_model(1, 3)) / 2,
			(line_model(2, 0) + line_model(2, 2)) / 2,
			(line_model(2, 1) + line_model(2, 3)) / 2,
			(line_model(3, 0) + line_model(3, 2)) / 2,
			(line_model(3, 1) + line_model(3, 3)) / 2;
		MatrixXd length_l(4, 1), c(3, 3);
		double ll1, ll2, ll3, ll4;

		ll1 = sqrt(pow((line_model(0, 0) - line_model(0, 2)), 2) +
			pow((line_model(0, 1) - line_model(0, 3)), 2));
		ll2 = sqrt(pow((line_model(1, 0) - line_model(1, 2)), 2) +
			pow((line_model(1, 1) - line_model(1, 3)), 2));
		ll3 = sqrt(pow((line_model(2, 0) - line_model(2, 2)), 2) +
			pow((line_model(2, 1) - line_model(2, 3)), 2));
		ll4 = sqrt(pow((line_model(3, 0) - line_model(3, 2)), 2) +
			pow((line_model(3, 1) - line_model(3, 3)), 2));
		

		/* length_l contains the length of all the lines */
		length_l << ll1,
			ll2,
			ll3,
			ll4;

		//  cout << length_l << endl;

		int line_rows = line_model.rows();
		dx = 0; dy = 0; da = 0;

		for (i = 0; i < line_rows; i++)
		{
			Z << line_model(i, 0), line_model(i, 2),
				line_model(i, 1), line_model(i, 3);

			T << line_model(i, 0), line_model(i, 0),
				line_model(i, 1), line_model(i, 1);

			dif = Z - T;

			z2 = dif / dif.norm();
			z = rot_v * z2;

			ui.row(i) << z(0, 1), z(1, 1);
			lr << (line_model.row(i)).head(2);
			//lr << line_model(i, Eigen::seq(1,2));
			lu << ui.row(i);
			//cout << lu << endl;
			r = (lr.array() * lu.array()).sum();
			ri(i, 0) = r;
			line_matrix.row(i) << line_model.row(i), midpi.row(i), (length_l(i, 0) / 2);


		}
		//cout <<  << endl;

		for (i = 0; i < max_iterations; i++)
		{
			rx += b(0);
			ry += b(1);
			// to ensure it remains inside 2 PI range
			//ra = findmod(ra + double(b(2)), 2 * pi);
			ra += b(2);
			x = (dis.array() * ( ang.array().cos()));
			y = (dis.array() * ( ang.array().sin()));
			Sens_to_robo_rot << cos(gamma), -sin(gamma), alpha,
				sin(gamma), cos(gamma), beta,
				0, 0, 1;
			robo_to_world_rot << cos(ra), -sin(ra), rx,
				sin(ra), cos(ra), ry,
				0, 0, 1;

			//robo_cord << x, y.block(0, 0, 200, 1), bias1.block(0, 0, 200, 1);
			// y, bias1);
			robo_cord << x,
				y,
				bias1;

			//<< robo_cord.rows() << robo_cord.cols() << endl;
			robo_cord1 = Sens_to_robo_rot * robo_cord;

			robo_cord_rc = robo_to_world_rot * robo_cord1;

			world_cord = robo_cord_rc.transpose();

			int n = world_cord.rows();
			int numLines = ui.rows();


			Yi.resize(numLines);

			int iter1, iter2;
			double vi;
			for (iter1 = 0; iter1 < n; iter1++) {
				for (iter2 = 0; iter2 < numLines; iter2++) {
					vi = (ui.row(iter2)).dot(world_cord.row(iter1).head(2));

					Yi(iter2) = ri(iter2, 0) - vi;

				}


				double min_index;
				double Yt;
				(Yi.array().abs().minCoeff(&min_index, &Yt));
				int min1 = static_cast<int>(min_index);
				Yt = Yi(min1);

				ind_dis_abs(iter1, 0) = min1;
				ind_dis_abs(iter1, 1) = Yt;
				ind_dis_abs(iter1, 2) = fabs(Yt);
			}






			VectorXd column = ind_dis_abs.col(2); // Extract the absolute distance value from 3rd column
			//double Median_Yt = median(column, column.rows()); // Calculate the median of the column

			// Removing the points which are greater than the median
			//VectorXd ind = (column.array() <= Median_Yt).cast<double>();
			VectorXd ind = (column.array() <= 100).cast<double>();
			//VectorXd ind = (column.array() <= 300).cast<double>();
			//cout << " index is " << ind << endl;



			world_cord_updated << world_cord, ind_dis_abs;



			MatrixXd ui_new(ind.size(), 2);
			//cout << ui_new.rows() << ui_new.cols() << endl;


			int count = 0;
			int iter3;
			for (iter3 = 0; iter3 < ind.size(); iter3++)
			{
				if (ind(iter3) == 1)
				{
					// TODO: check if there is a difference if assigned to a diff variable 
					world_cord_updated.row(count) = world_cord_updated.row(iter3);
					ui_new.row(count) = ui.row(ind_dis_abs(iter3, 0));
					count++;
				}
			}
			/*int index_count = 1;
			for (iter1 = 0, iter1 < count; iter1++){
				dis_mid(iter1) = sqrt((world_cord_updated(iter1,1)- \
				line_matrix(world_cord_updated(iter1,4),5))^2+((world_cord_updated(iter1,2)-line_matrix(world_cord_updated(iter1,4),6))^2));
				if(dis_mid(iter1) <= (line_matrix(world_cord_updated(iter1,4),7))){
					//world_cord_updated(index_count, :) = [world_cord_updated(ii, 1:5) U(world_cord_updated(ii,4), 1) U(world_cord_updated(ii,4), 2)];
					index_count = index_count + 1;
				}
			}*/





			int points = count;
			//world_cord_updated = world_cord_updated.topRows(count);
			MatrixXd world_cord_updated1 = world_cord_updated.block(0, 0, points, 6);

			MatrixXd ui_new1 = ui_new.block(0, 0, points, 2);
			//cout << "Points" << points << endl;




			MatrixXd world_cord_final(points, 8);
			world_cord_final << world_cord_updated1, ui_new1;

			// cout << world_cord_final << endl;



			MatrixXd Xi1(points, 1), Xi2(points, 1), Xi3(points, 1);


			Xi1 = world_cord_final.col(6);


			Xi2 = world_cord_final.col(7);


			Xi3 = MatrixXd::Zero(Xi2.rows(), Xi2.cols());

			int nn;
			for (nn = 0; nn < world_cord_final.rows(); nn++)
			{

				MatrixXd temp1(1, 2);
				temp1 << Xi1(nn, 0), Xi2(nn, 0);
				MatrixXd temp2(2, 2);
				temp2 << 0, -1, 1, 0;
				MatrixXd temp3(2, 1);
				temp3 << world_cord_final(nn, 0) - world_cord_final.col(0).mean(), world_cord_final(nn, 1) - world_cord_final.col(1).mean();
				Xi3(nn, 0) = (temp1 * temp2 * temp3)(0, 0);
			}


			MatrixXd A(Xi1.rows(), 3);
			A << Xi1, Xi2, Xi3;

			MatrixXd Y = world_cord_final.col(4);

			b = (A.transpose() * A).inverse() * A.transpose() * Y;




			double s2;


			s2 = (world_cord_final.col(4) - (A * b)).dot((world_cord_final.col(4) - (A * b))) / (points - 4);
			c = s2 * ((A.transpose() * A).inverse());
			dx += b(0);
			dy += b(1);	
			//da += findmod(b(2), 2 * (pi / 180));
			da += b(2);
			bool condition = (sqrt((b(0) * b(0)) + (b(1) * b(1))) < 1 && abs(b(2)) < 0.01*M_PI/180);
			double cond_value = sqrt((b(0) * b(0)) + (b(1) * b(1)));
			//cout << cond_value << endl;
			//cout << "Condition " << condition << endl;
			//cout << "iteration value = " << i << endl;

			

			if (fabs(dx) > 100 || fabs(dy) > 100 ||  fabs(da) > 60*M_PI/180){
				cout << "THIS DOES NOT WORK" << endl;
				dx = 0;
				dy = 0;
				da = 0;
				c << 1 , 0 , 0, 0, 1, 0, 0, 0, 0.1;
				i = max_iterations;
				
			}



			if (condition) {
				//da = da * (180 / pi);	
				//cout << "Final iteration value = " << i << endl;
				//cout << "dx is " << dx << endl;
				//cout << " dy is " << dy << endl;
				//cout << " da is " << da  << endl;
				//cout << dx + rx <<" "<<dy + ry<<endl;
				//cout << "*************************************" <<endl;

				
				i = max_iterations; //exit condition due to convergance 
				b << 0, 0, 0;


			}




			// set all elements of Xi1 to zero
			Xi1.setZero();

			// set all elements of Xi2 to zero
			Xi2.setZero();

			// set all elements of Xi3 to zero
			Xi3.setZero();

			// set all elements of A to zero
			A.setZero();


		}

		
		if(isDoubleNaN(dx)){
		dx = 0;
		c << 1 , 0 , 0, 0, 1, 0, 0, 0, 0.1;}
		if(isDoubleNaN(dy)){
		dy = 0;
		c << 1 , 0 , 0, 0, 1, 0, 0, 0, 0.1;}
		if(isDoubleNaN(da)){
		da = 0;
		c << 1 , 0 , 0, 0, 1, 0, 0, 0, 0.1;}

		
		/*
		dx = 0;
		dy = 0;
		da = 0;
		c << 1000 , 1 , 1, 1, 1000, 1, 1, 1, 1000;
		cout << endl<< endl<< endl;
		*/
		



		//da = da * (180 / pi);
		//scan_match_done = 1;
		//cout << "dx is " << dx << " x is " << dx<< endl;
		//cout << " dy is " << dy<< " y is " << dy << endl;
		//cout << " da is " << da * 180 / M_PI <<"a is " << da* 180 / M_PI<<endl;
		//cout << dx + rx <<" "<<dy + ry<<endl;
		//cout << " uncertainty is " << c << endl;
		//cout << "*************************************" << endl;
		b << 0, 0, 0;
		//cox_output_t.x = dx +  odo_x; 
		//cox_output_t.y = dy + odo_y;   
		//cox_output_t.ang = findmod(da + odo_a, 2*M_PI);
		if(dx == 0 && dy == 0 && da == 0)
		{
			cox_output_t.x = 0; 
			cox_output_t.y = 0;   
			cox_output_t.ang = 0;
			cox_output_t.Cov = c;
		}
		cox_output_t.x = dx +  odometry_t.x; 
		cox_output_t.y = dy + odometry_t.y;   
		cox_output_t.ang = findmod(da + odometry_t.ang, 2*M_PI);
		cox_output_t.Cov = c;
		//cout <<"cox x" << cox_output_t.x << " cox y" << cox_output_t.y << "cox angle" << cox_output_t.ang <<endl;
		scan_match_done = 1;

}
}
}



