//FOR USE WITH GPS ONLY

#include <Eigen/Geometry>
#include "tw_odom.hpp"
#include <string>
#include <iostream>


namespace tw_f
{
twNode::twNode(ros::NodeHandle &nh)
{

  	//Get data about node and topic to listen
  	std::string quadPoseTopic;
  	double tmax, pubRate;
  	quadName = ros::this_node::getName();
  	ros::param::get(quadName + "/quadPoseTopic", quadPoseTopic);
  	ros::param::get(quadName + "/maxTW",tmax);
  	ros::param::get(quadName + "/mass",quadMass);
  	ros::param::get(quadName + "/pubRate", pubRate);
  	ros::param::get(quadName + "/groundLevel",floorL);
  	throttleMax = tmax*9.81;
    twCounter=0;

  	lastGpsTime=0.0;
  	lastJoyTime=0.0;

  	twCounter=0;
  	xCurr(2)=0;

  	//should be a const but catkin doesn't like scoping it
  	pi = std::atan(1.0)*4;

  	kfInit=false; //KF will need to be initialized
  	throttleSetpoint = 9.81/throttleMax; //the floor is the throttle

 	bool useUDP = false;
 	ros::param::get(quadName + "/useUDP",useUDP);

	quaternionSetpoint.x() = 0;
	quaternionSetpoint.y() = 0;
	quaternionSetpoint.z() = 0;
	quaternionSetpoint.w() = 1;

	gps_sub_ = nh.subscribe(quadPoseTopic, 1, &twNode::gpsCallback,
							this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
  	thrustSub_ = nh.subscribe("mavros/setpoint_attitude/att_throttle", 1,
							&twNode::throttleCallback,this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	attSub_ = nh.subscribe("mavros/setpoint_attitude/attitude", 1,
							&twNode::attSetCallback,this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
  	joy_sub_ = nh.subscribe("joy",1,&twNode::joyCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
  	twPub_ = nh.advertise<tw_filter::twUpdate>("ThrustToWeight",1);
  	timerPub_ = nh.createTimer(ros::Duration(1.0/pubRate), &twNode::timerCallback, this, false);
}


void twNode::timerCallback(const ros::TimerEvent &event)
{
	if(twCounter<=10)
		{return;}

	double meanTW=0.0;

	for(int ij=0;ij<twCounter;ij++)
	{
    //std::cout << "val: " << twStorage(ij) << std::endl << 
    //  "sum: " << (1.0/twCounter)*twStorage(ij) << std::endl;
		meanTW = meanTW + (1.0/twCounter)*twStorage(ij);
	}

	double battstat(100.0*(1.0-(1.80-meanTW)/0.4));

	tw_filter::twUpdate tw_msg;
	tw_msg.rosTime = tCurr();
	tw_msg.estimatedTW = meanTW;
	twPub_.publish(tw_msg);
	ROS_INFO("Updating T/W to %f. Battery at approximately %f%%",meanTW,battstat);
	twCounter=0;
}


void twNode::throttleCallback(const std_msgs::Float64::ConstPtr &msg)
{
	if(!kfInit)
		{return;}
	lastJoyTime = tCurr();
  	//if on the ground / else if taken off
  	if(xCurr(2)<floorL)
  	{
		throttleSetpoint = 9.81/throttleMax; //the floor is the throttle
  	}else{
		throttleSetpoint = throttleMax * msg->data;
  	}
}
void twNode::attSetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if(!kfInit)
		{return;}	
  	if(xCurr(2)<floorL)
  	{
		quaternionSetpoint.x() = 0;
		quaternionSetpoint.y() = 0;
		quaternionSetpoint.z() = 0;
		quaternionSetpoint.w() = 1;
  	}else{
		quaternionSetpoint.x() = msg->pose.orientation.x;
		quaternionSetpoint.y() = msg->pose.orientation.y;
		quaternionSetpoint.z() = msg->pose.orientation.z;
		quaternionSetpoint.w() = msg->pose.orientation.w;
  	}
}


void twNode::gpsCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	double tNow = tCurr();

  Eigen::Matrix<double,6,1> zMeas;
 	if(kfInit)  //only filter if initPose_ has been set
  	{
//      ROS_INFO("Pose received");
  		Eigen::Matrix3d RR(quaternionSetpoint);
  		Eigen::Vector3d uvec = RR*Eigen::Vector3d(0.0,0.0,throttleSetpoint);
  		
	  	//T/W filter

	  	double dt = tNow - lastGpsTime;
		kfTW_.processUpdate(dt,uvec);
		Eigen::Matrix<double,7,1> xStateAfterProp=kfTW_.getState();
		//ROS_INFO("T/W after propagation: %f",xStateAfterProp(6));
		//update kfTW_
		KalmanTW::Measurement_t measM;
		measM(0) = msg->pose.pose.position.x;
		measM(1) = msg->pose.pose.position.y;
		measM(2) = msg->pose.pose.position.z;
		measM(3) = msg->twist.twist.linear.x;
		measM(4) = msg->twist.twist.linear.y;
		measM(5) = msg->twist.twist.linear.z;
		kfTW_.measurementUpdate(measM);
		xCurr=kfTW_.getState();

	  	if(xCurr(2)>=(floorL+0.10) && isArmed && twCounter<2000)
	  	{
//        std::cout << twCounter << std::endl;
        twStorage(twCounter)=xCurr(6)*throttleMax/9.81;  //throttleMax=9.81*tw[0]
			  twCounter++;
		  }
	}
	else //run intitialization
	{
		Eigen::Vector3d p0(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
		ROS_INFO("initpose: %f %f %f",p0(0),p0(1),p0(2));

		// Initialize Kalman Filter with T/W modification
		KalmanTW::ProcessCov_t procNoise = (Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.000003)).asDiagonal();
		KalmanTW::MeasurementCov_t measNoise = 1e-1 * Eigen::Matrix<double,6,6>::Identity();
		KalmanTW::StateCov_t initCov = Eigen::Matrix<double,7,7>::Identity();
		initCov(6,6)=0.1;
		KalmanTW::State_t initState;
		initState << p0(0), p0(1), p0(2), 0.0, 0.0, 0.0, 1.0;
		xCurr = initState;
		kfTW_.initialize(initState,initCov,procNoise,measNoise);
		Eigen::Matrix<double,7,1> checkvec;

		kfInit=true;
	}

  	//lastRosTime is the ROS timestamp corresponding to the last time published
  	lastGpsTime = tNow;
  	return;
}


double twNode::tCurr()
{
	return (ros::Time::now()).toSec();
}



void twNode::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  	if(msg->buttons[1]==1 || msg->buttons[2]==1 || msg->buttons[3]==1)
  	{
		isArmed=true;
  	}
  	else if(msg->buttons[0]==1)
  	{
		isArmed=false;
  	}
}

} //end namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tw_f");
  ros::NodeHandle nh;

  try
  {
	tw_f::twNode tw_ffn(nh);
	ros::spin();

  }
  catch(const std::exception &e)
  {
	ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
	return 1;
  }
  return 0;
}
