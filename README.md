This project focuses on the development of an Autonomous Mobile Robot (AMR) equipped with odometry and LIDAR sensors. The system's navigation and localization functionalities are implemented using the COX algorithm and Kalman Filter.

Features
Odometry Integration: Utilizes wheel encoder data to estimate the robot's position over time.
LIDAR Sensing: Employs LIDAR sensors for environmental mapping and obstacle detection.
COX Algorithm: Implements the COX algorithm for data association in the context of simultaneous localization and mapping (SLAM).
Kalman Filter: Applies the Kalman Filter for sensor fusion and state estimation, enhancing the accuracy of the robot's perceived position.
File Structure
Camera.h and camera.cpp: Handle camera-related functionalities.
cox.h and cox.cpp: Contain the implementation of the COX algorithm.
global.h: Defines global variables and configurations.
lidar.cpp: Manages LIDAR sensor data processing.
lidar_server.h: Interfaces with the LIDAR hardware.
main.cpp: The main entry point of the application.
odo_speed_profile.cpp and odo_speed_profile.h: Manage odometry speed profiling.
position_feedback_controller.cpp: Implements the position feedback control mechanism.
spi_com.h: Handles SPI communication protocols.
