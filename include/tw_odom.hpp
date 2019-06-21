#ifndef tw_f_NODE_HPP_
#define tw_f_NODE_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include "filterTW.h"
#include <tw_filter/twUpdate.h>

namespace tw_f
{
class twNode
{
 public:
  twNode(ros::NodeHandle &nh);

  void gpsCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void attSetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void throttleCallback(const std_msgs::Float64::ConstPtr &msg);
  void timerCallback(const ros::TimerEvent &event);
  double tCurr();

 private:

  KalmanTW kfTW_;
  KalmanTW::State_t xCurr;
  ros::Publisher twPub_;
  std::string quadName;
  ros::Subscriber gps_sub_, joy_sub_, attSub_, thrustSub_;
  ros::Timer timerPub_;
  Eigen::Quaterniond quaternionSetpoint;
  double throttleSetpoint, throttleMax, quadMass, pubRate, tmax, tmin;
  bool kfInit, isArmed, useCommQuat_;
  double pi, floorL;
  double lastGpsTime, lastJoyTime;
  //Preallocate a larger array than necessary to avoid overhead due to resizing/copying
  Eigen::Matrix<double,200,1> twStorage;
  int twCounter;
};

} // tw_f

#endif // tw_f_NODE_HPP_
