#include <geometry_msgs/Twist.h>
#include <nifti_robot_driver_msgs/Tracks.h>
#include <ros/ros.h>

double robot_width = 0.6;  // m
double vel_max_tracks = 10.0;  // m
ros::Publisher vel_pub;
void callback(const geometry_msgs::Twist::ConstPtr& msg) {

  // Convert speeds to track speeds
  double linear_vel = msg->linear.x;
  double angular_vel = msg->angular.z;

  double d = robot_width / 2;

  nifti_robot_driver_msgs::Tracks tracks_cmd;
  tracks_cmd.left = linear_vel - (d * angular_vel);
  tracks_cmd.right = linear_vel + (d * angular_vel);

  /// < scale the commands by maintaining the orginal velocities ratio (i.e. maintainingt the curvature radius)
  if (fabs(tracks_cmd.left) > vel_max_tracks) {
    double scale = fabs(vel_max_tracks / tracks_cmd.left);
    tracks_cmd.left *= scale;
    tracks_cmd.right *= scale;
  }

  if (fabs(tracks_cmd.right) > vel_max_tracks) {
    double scale = fabs(vel_max_tracks / tracks_cmd.right);
    tracks_cmd.left *= scale;
    tracks_cmd.right *= scale;
  }

  std::cout << "Sending velocities " << linear_vel << "\n";
  vel_pub.publish(tracks_cmd);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_conversion");
  ros::NodeHandle nh;
  vel_pub = nh.advertise<nifti_robot_driver_msgs::Tracks>("tracks", 1);
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &callback);
  ros::spin();
}