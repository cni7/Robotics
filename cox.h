#include <eigen/Eigen/Dense>
using namespace Eigen;
void *scan_match(void *);
double findmod(double x, double y);
extern int scan_match_done;
extern int initiate_kalman;
extern struct position_s cox_output_t;
extern double odo_x;
extern double odo_y;
extern double odo_a;
extern Matrix3d covariance;
