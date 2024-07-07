#include <unistd.h>
#include <pthread.h>
#include <iostream>

#include "global.h"
//#include ”camera.h”
#include "lidar_server.h"
#include "Camera.h"
#include "cox.h"
#include "odo_speed_profile.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>




#include <fstream>
#include <chrono>
#include <iomanip>






static double old_odo_x, old_odo_y, old_odo_a, old_cox_a, old_cox_x, old_cox_y, old_kal_x, old_kal_y, old_kal_a;


extern "C" {
#include "spi_com.h"
}


MotorDataType MotorData;

int Need_Laser_Data = 1;

using namespace std;


// Open the log file
std::ofstream logFile0("log_odo_cox_kal.txt");
std::ofstream logFile("log_Odometry.txt");
std::ofstream logFile2("log_Cox.txt");
std::ofstream logFile3("log_Kalman.txt");
int count_mine = 0;
auto startTime = std::chrono::high_resolution_clock::now();


int main(void)
{
	pthread_t thread1, thread2, thread3;
	int p1, p2, p3, p4, i;
	char key;
	p2 = pthread_create(&thread1, NULL, camera, NULL);
	p1 = pthread_create(&thread2, NULL, Lidar_Server, NULL);
    p3 = pthread_create(&thread3, NULL, scan_match, NULL);
	p4 = pthread_create(&thread3, NULL, set_speed, NULL);
	if(p1 | p3 | p4 | p2)	
	//if(p1 | p4 | p2)	
	{
		cout << "threads missing";
	}
	MotorData.Set_Speed_M1 = 0;
	MotorData.Set_Speed_M2 = 0;
	Send_Read_Motor_Data(&MotorData);


	logFile  << "Time_s "<< "ODO_X_mm " << "ODO_Y_mm "  << "ODO_ANG " << "C_00 " << "C_01 "<< "C_02 " << "C_10 " << "C_11 "<< "C_12 " <<"C_20 " << "C_21 "<< "C_22 " <<std::endl;
        
	while(1){
			
			//cout << "MAIN MAIN MAIN odo is :" << odometry_t.x << " " <<odometry_t.y << " " << odometry_t.ang<<endl;
			
		       auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime);
        		float timeInSeconds = (float)elapsedTime.count() / 1000.0;
			
		        int remainder = elapsedTime.count() % 1000;
        		bool isCloseToMultiple = (remainder <= 100 || remainder >= 900);			
			

			
		        
		        //Write the time and value to the log file
		        logFile  << timeInSeconds  << " " << odometry_t.x  << " " << odometry_t.y  <<" " << 
			odometry_t.ang <<" "<< odometry_t.Cov(0,0)<<" "<< odometry_t.Cov(0,1)<<" "<< odometry_t.Cov(0,2)<<" "<<     
			odometry_t.Cov(1,0)<<" "<< odometry_t.Cov(1,1)<<" "<< odometry_t.Cov(1,2)<<" "<<      
			odometry_t.Cov(2,0)<<" "<< odometry_t.Cov(2,1)<<" "<< odometry_t.Cov(2,2)<<" "<< std::endl;

		        logFile2  << timeInSeconds  << " " << cox_output_t.x  << " " << cox_output_t.y  <<" " << 
			cox_output_t.ang <<" "<< cox_output_t.Cov(0,0)<<" "<< cox_output_t.Cov(0,1)<<" "<< cox_output_t.Cov(0,2)<<" "<<     
			cox_output_t.Cov(1,0)<<" "<< cox_output_t.Cov(1,1)<<" "<< cox_output_t.Cov(1,2)<<" "<<      
			cox_output_t.Cov(2,0)<<" "<< cox_output_t.Cov(2,1)<<" "<< cox_output_t.Cov(2,2)<<" "<< std::endl;

		        logFile3  << timeInSeconds  << " " << correction.x  << " " << correction.y  <<" " << 
			correction.ang <<" "<< correction.Cov(0,0)<<" "<< correction.Cov(0,1)<<" "<< correction.Cov(0,2)<<" "<<     
			correction.Cov(1,0)<<" "<< correction.Cov(1,1)<<" "<< correction.Cov(1,2)<<" "<<      
			correction.Cov(2,0)<<" "<< correction.Cov(2,1)<<" "<< correction.Cov(2,2)<<" "<< std::endl;
			if(kalman_count)
			{
				logFile3 <<  "--------------------------------------------------- "<< endl;
				logFile2 << "--------------------------------------------------- "<<endl;
				logFile << "---------------------------------------------------- "<< endl;
				logFile3 <<  "----------------------KALMAN DONE----------------------------- "<< endl;
				logFile2 << "-----------------------KALMAN DONE---------------------------- "<<endl;
				logFile << "------------------------KALMAN DONE---------------------------- "<< endl;
				logFile0 << "---------------------------------------------------- "<< endl;
				logFile0 <<  "----------------------KALMAN DONE----------------------------- "<< endl;
				logFile0  << timeInSeconds  << "  ODO_X:" << odometry_t.x << "  COX_X:" << cox_output_t.x << "  KAL_X:" << correction.x << std::endl;
			logFile0  << timeInSeconds  << "  ODO_Y:" << odometry_t.y << "  COX_Y:" << cox_output_t.y << "  KAL_Y:" << correction.y << std::endl << std::endl;
	
				
			}
			logFile0  << timeInSeconds  << "  ODO_X:" << odometry_t.x<<endl; // << "  COX_X:" << cox_output_t.x << "  KAL_X:" << correction.x << std::endl;
			logFile0  << timeInSeconds  << "  ODO_Y:" << odometry_t.y<<endl << endl <<endl;  //<< "  COX_Y:" << cox_output_t.y << "  KAL_Y:" << correction.y << std::endl << std::endl;
			
			
			

			
			//cout <<"MAIN cox" <<cox_output_t.x<< std::endl; 
			
			
		

	}
		
 	logFile.close();
	logFile2.close();
	logFile3.close(); 
	return 0;	
}
