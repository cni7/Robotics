#include <eigen/Eigen/Dense>

using namespace Eigen;


struct position_s
{
	float x;
	float y;
	float ang;
	Matrix3d Cov;

};

struct speed_s
{
	float velocity;
	float angular_velocity;
};

struct motor_speed_s
{
	float v_l;
	float v_r;
};

extern position_s odometry_t;
extern int odometry_done;
extern position_s correction;
extern int kalman_count;
void *set_speed(void *);