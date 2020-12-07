#include <ros/ros.h>
#include<cmath>
using namespace std;
/*
A toy example of pathchecker Node.
Input:
actual_path: the actual path of satlet from Gazebo.
Path should be a 2-D array. First dimension is the length of time steps. Second dimension is the num of dimensions considered in the motion.
planned_path: the planned path of stalet from path planning node.
Output: the mean of deviation between actual path and planned path in the last few steps.
*/

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "abp_sim_pathcheck");

    ros::NodeHandle nh;

    int path_length = 10; // pre-defined length of path.
    int num_dimension = 3;

    double actual_path[path_length][num_dimension];
    double planned_path[path_length][num_dimension];
    // Assign some value 
    for (int i = 0; i < path_length; i++) {
        for (int j = 0; j < num_dimension; j++) {
            actual_path[i][j] = 0;
            planned_path[i][j] = -1;
        }
    };

    // Calculate the max deviation
    double sum_error = 0;
    for (int i = 0; i < path_length; i++) {
        double current_error = 0;
        for (int j = 0; j < num_dimension; j++) {
            current_error += (actual_path[i][j] - planned_path[i][j])*(actual_path[i][j] - planned_path[i][j]);
        }
        sum_error += current_error;
    };
    
    double aveg_error = (sum_error / path_length);
    ROS_INFO("Diff is [%f]", aveg_error);
    return aveg_error;
}