#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <map>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include <ecl/threads.hpp>

using namespace std;
using Client =  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ;

ros::Publisher *cmdVelPubPtr;
Client *actionClient;
geometry_msgs::Point current_point;
geometry_msgs::Pose2D base_p_;
ecl::Thread thread_;

geometry_msgs::Point setPoint(double _x, double _y, double _z);
geometry_msgs::Quaternion setQuaternion(double _angleRan);
vector<geometry_msgs::Pose> init_goalList();
visualization_msgs::Marker init_markers(const vector<geometry_msgs::Pose>  &wayPoints);
void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal);
void shutdown(int sig);
void baseposeCallback(geometry_msgs::Pose2D bp);


geometry_msgs::Point setPoint(double _x, double _y, double _z)
{
	geometry_msgs::Point m_point;
	m_point.x = _x;
	m_point.y = _y;
	m_point.z = _z;
	return m_point;
}

geometry_msgs::Quaternion setQuaternion(double _angleRan)
{
	geometry_msgs::Quaternion m_quaternion;
	m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan);
	return m_quaternion;
}

vector<geometry_msgs::Pose> init_goalList()
{
	//Create a list to hold the target quaternions (orientations)
	vector<geometry_msgs::Pose> wayPoints;
	geometry_msgs::Pose pose ;

	int points_size = 0;
	float x = 0.0;
	float y = 0.0;
	float th = 0.0;
	string ParamName;
	ros::param::get("/navi_scene/points_size",points_size);
	for(int i = 1; i < points_size+1; i++){
		ParamName = "/navi_scene/point_" + to_string(i)  + "/x";
		ros::param::get(ParamName,x);
		//ROS_INFO("point_x: %f",x);
		ParamName = "/navi_scene/point_" + to_string(i)  + "/y";
		ros::param::get(ParamName,y);
		//ROS_INFO("point_y: %f",x);
		ParamName = "/navi_scene/point_" + to_string(i)  + "/th";
		ros::param::get(ParamName,th);
		ROS_INFO("point_%d: %f  %f  %f",i,x,y,th);
		pose.position = setPoint(x,y, 0);
		pose.orientation = setQuaternion(th);
		wayPoints.push_back(pose);
	}
	return wayPoints;
}


visualization_msgs::Marker init_markers(const vector<geometry_msgs::Pose>  &wayPoints)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();

	marker.ns = "waypoints";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(); //0 is forever
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 1.0;
	marker.color.g = 0.7;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	for (const auto &point : wayPoints){
		marker.points.push_back(point.position);
	}
	return marker;
}

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("New goal Received");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	//ROS_INFO("Got base_position of Feedback");
	current_point.x = feedback->base_position.pose.position.x;
	current_point.y = feedback->base_position.pose.position.y;
	current_point.z = feedback->base_position.pose.position.z;
	//ROS_INFO("x y: %f  %f  %f ",current_point.x,current_point.y,current_point.z);
}

void baseposeCallback(geometry_msgs::Pose2D bp)
{
	base_p_.x = bp.x;
	base_p_.y = bp.y;
	base_p_.theta = bp.theta;
    //ROS_INFO("x y: %f  %f  %f ",base_p_.x,base_p_.y,base_p_.theta);
}

void shutdown(int sig)
{
     actionClient->cancelAllGoals();
   	ros::Duration(2).sleep(); 

	cmdVelPubPtr->publish(geometry_msgs::Twist());
	ros::Duration(1).sleep(); 
	ROS_INFO("Stopped the robot!");
	ros::shutdown();
}

void thread_func()
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navi_scene_deploy");
	ros::NodeHandle node;
	ros::Rate loop_rate(10);
	signal(SIGINT, shutdown);
	ROS_INFO("navi_scene Deploy Node Start...");
	ros::Subscriber sub_basepos = node.subscribe("basepose",1000,baseposeCallback);
	thread_.start(ecl::generateFunctionObject(thread_func));

	ROS_INFO("Waiting for move_base action server...");
	actionClient = new Client("move_base", true);
	if (!actionClient->waitForServer(ros::Duration(60)))
	{
		ROS_WARN("Can't connected to move base server");
		return 1;
	}
	ROS_INFO("Connected to move base server");


	ROS_INFO("Starting path planning and tracking...");
	vector<geometry_msgs::Pose> wayPoints = init_goalList();
	visualization_msgs::Marker  marker = init_markers(wayPoints);
	ros::Publisher cmdVelPub = node.advertise<geometry_msgs::Twist>("/raw_cmd_vel", 5);
	cmdVelPubPtr = &cmdVelPub;
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

	move_base_msgs::MoveBaseGoal goal;

	//Cycle through the four waypoints
	string taskParamName = "/usr_task";
	int task=0;

	while (1)
	{
		/* code */
		ROS_INFO("Waiting for usr_task...");
		do {
			ros::param::get(taskParamName,task);
			if(task == 2) break;
			ros::Duration(1).sleep(); 
		}while ( 1 );

		for(int pointIndex=0; pointIndex < wayPoints.size(); ++pointIndex){
			//ROS_INFO("start.............");
			marker_pub.publish(marker);
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose = wayPoints[pointIndex];
			actionClient->sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);

			if(actionClient->waitForResult(ros::Duration(900))){
				if (actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					ROS_INFO("the %d wayPoint arrived!", pointIndex);
					if(pointIndex == wayPoints.size() - 1) ros::param::set("/usr_task",0);
				}
			}else{
				actionClient->cancelAllGoals();
				ROS_WARN("Timed out achieving goal!!!");
			}
		}
		ros::Duration(1).sleep();

	}

	ROS_INFO("Path planning and tracking  end...");
	return 0;
}
