#include "global.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <algorithm>
#include <vector>
#include <eigen/Eigen/Dense>

#include "cox.h"
#include "lidar_server.h"

#include <fstream>

#define HEADER_LEN 5
#define DATA_LEN 4096
#define TCP_IP "127.0.0.1"
#define TCP_PORT 9887
#define BUFFER_SIZE 1024
using namespace std;


Eigen::MatrixXd dis(1, 500);
Eigen::MatrixXd ang(1, 500);
//extern int Need_Laser_Data;
int Array_full_flag = 0;

std::ofstream logFile4("log_lidar.txt");

void *Lidar_Server(void *){
	
	int Index = 0 ;
	int Received_Bytes = 0 ;
	char recv_buffer[10];
	int i;
    int sockfd, newsockfd,portno;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    portno = 9888;
    int iQuality;
    int iDistance;
    int iAngle;
    int Counter = 0;
    float pi = 3.1416;
    

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
		puts("ERROR opening socket");
	else
		puts("open socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        perror("ERROR on binding");
    else
        puts("binding Sucessful");
    
    int sockfd1;
    
    char buffer[BUFFER_SIZE];

    sockfd1 = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd1 < 0) {
        perror("ERROR opening socket");
        exit(1);
    }

    
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(TCP_IP);
    serv_addr.sin_port = htons(TCP_PORT);

    if (connect(sockfd1, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR connecting");
        exit(1);
       }
     else
       {
        puts("Connected");
       }

    char message[] = "\x10\x00";
    if (send(sockfd1, message, strlen(message), 0) < 0) {
        perror("ERROR sending message");
        exit(1);
    }

    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) 
    {
        perror("ERROR on accept");
        exit(1);
    }
    else
	{
		puts("Socket Accepted");
	}  
	while(1){
		
		Received_Bytes = recv(newsockfd, recv_buffer, 5, 0);
		if(recv_buffer[0] == 0xA5){
			Received_Bytes = recv(newsockfd, recv_buffer, 5, 0);
			if(scan_match_done == 1) {
				Array_full_flag = 0;
                                 
			} 
			iQuality = recv_buffer[0] >> 2;
		    iAngle = (recv_buffer[1] >> 1) + (recv_buffer[2] << 8);
			iDistance = (recv_buffer[3]) + (recv_buffer[4] << 8);  
			if (scan_match_done == 0 && Array_full_flag == 0) { 
			        // Take care of multipel times
				if ((iQuality >=15 ) && (iDistance != 0) ) {
					// Todo for you! check quality and store data
					//cout <<"Counter = "<< Counter <<endl;	
					dis(0, Counter)  = (double)(iDistance / 4.0);
					ang(0, Counter) = (((double)iAngle) * 0.000136353);
					logFile4 << "15" << " " << ang(0, Counter) << " " << dis(0, Counter) << std::endl;
					Counter++;
					
					
					
					
					if(Counter == 500){
						Array_full_flag = 1;
						Counter = 0;
  						
						//cout << ang <<" "<< dis<< endl;
					}								
				}
			}
		}
	}
	logFile4.close();
	close(newsockfd);
    close(sockfd);
    
}


