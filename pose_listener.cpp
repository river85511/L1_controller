#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <time.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"
//#include <Quaternion.h>

#define PI 3.14159265


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

class PoseSubscriiber
{
public:
  PoseSubscriiber(): L(0.175), Lfw(3), Vcmd(1), lfw(0.0875), Kp(0.2),reached_tag(0)
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Float32MultiArray>("l1control", 1000);///

    //Topic you want to subscribe
    sub_ = n_.subscribe("poseupdate", 1000, &PoseSubscriiber::odom_callback, this);
	waypoint_sub = n_.subscribe("move_base/TrajectoryPlannerROS/global_plan", 1000, &PoseSubscriiber::waypoint_callback, this);

	timer1 = n_.createTimer(ros::Duration(0.02), &PoseSubscriiber::control_loop, this); // Duration(0.02) -> 50Hz
	current_time = ros::Time::now();
	last_time = ros::Time::now();
  }

  void odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
  {
		current_time = ros::Time::now();

		mypose_now = *pose;
		double x_now = mypose_now.pose.pose.orientation.x;
		double y_now = mypose_now.pose.pose.orientation.y;
		double z_now = mypose_now.pose.pose.orientation.z;
		double w_now = mypose_now.pose.pose.orientation.w;
		
		tf::Quaternion q_now(x_now,y_now,z_now,w_now);
		tf::Matrix3x3 quaternion_now(q_now);
		quaternion_now.getRPY(roll_now, pitch_now, yaw_now);

		double x_last = mypose_last.pose.pose.orientation.x;
		double y_last = mypose_last.pose.pose.orientation.y;
		double z_last = mypose_last.pose.pose.orientation.z;
		double w_last = mypose_last.pose.pose.orientation.w;

		tf::Quaternion q_last(x_last,y_last,z_last,w_last);
		tf::Matrix3x3 quaternion_last(q_last);
		quaternion_last.getRPY(roll_last, pitch_last, yaw_last);


		double delta_time = (current_time - last_time).toSec();

		double delta_x =  (mypose_now.pose.pose.position.x - mypose_last.pose.pose.position.x);
		double delta_y =  (mypose_now.pose.pose.position.y - mypose_last.pose.pose.position.y);
		double delta_theta = (yaw_now - yaw_last);

		velocity_x = delta_x/delta_time;
		velocity_y = delta_y/delta_time;
		double velocity_theta = delta_theta/delta_time;


		//ROS_INFO("I heard: position_x = %f\tposition_y = %f\tposition _theta = %f\n", mypose_now.pose.pose.position.x, mypose_now.pose.pose.position.y, mypose_now.pose.pose.position.z);
		//ROS_INFO("I heard: veloctiy_x = %f\tveloctiy_y = %f\tvelocity_theta = %f\n", velocity_x, velocity_y, velocity_theta);
    //.... do something with the input and generate the output...

		mypose_last = *pose;
		last_time = ros::Time::now();

  }

	void waypoint_callback(const nav_msgs::Path::ConstPtr& waypoint){
			path = *waypoint;
			
			double car_x = mypose_now.pose.pose.position.x;
			double car_y = mypose_now.pose.pose.position.y;
			double car_theta = yaw_now;

			double car_heading_vect[2];
				   car_heading_vect[0] = (car_x - cos(yaw_now)) - car_x;
				   car_heading_vect[1] = (car_y - sin(yaw_now)) - car_y;

			double car_2_path_vect[2];

			double distance;

			for(int i =0; i< path.poses.size(); i++){
					double path_x = path.poses[i].pose.position.x;
					double path_y = path.poses[i].pose.position.y;
				
				    distance = sqrt( (car_x-path_x)*(car_x-path_x) + (car_y-path_y)*(car_y-path_y) );
					//ROS_INFO("yaw_now: %f", car_theta);
					if(distance >=  0.5){
						car_2_path_vect[0] = path_x - car_x;
						car_2_path_vect[1] = path_y - car_y;
						//ROS_INFO("Point Found! : [x = %f\ty= %f] " ,path_x,path_y);
							break;					
					}					
			}
			
			if(distance < 0.5){
				eta = 0;

				reached_tag = 1;
			}else{
				double path_x_from_car = cos(yaw_now)*(car_2_path_vect[0])+sin(yaw_now)*(car_2_path_vect[1]);
				double path_y_from_car = -sin(yaw_now)*(car_2_path_vect[0])+cos(yaw_now)*(car_2_path_vect[1]);
				
				eta = -atan2(path_y_from_car,path_x_from_car);
				
				reached_tag = 0;
			}


			//ROS_INFO("eta = %f",eta);
			//for(int i =0; i< path.poses.size(); i++){
			//ROS_INFO(" Point: %d [x = %f\ty= %f\tz = %f]",i,path.poses[i].pose.position.x,path.poses[i].pose.position.y,path.poses[i].pose.position.z);
			//}
	}

	void control_loop(const ros::TimerEvent&){
		//ROS_INFO("I LOVE YOU!");
	
		std_msgs::Float32MultiArray msg;		


		if(reached_tag == 0){
        steering = -atan((L*sin(eta))/(Lfw/2+lfw*cos(eta)));
        double v = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);  
		u = Kp*(Vcmd - v);
		}else if(reached_tag == 1){
			steering = 0;
			u = 0;
		}
		
        //ROS_INFO("velocity_x : %f\tvelocity_y : %f\n", velocity_x, velocity_y);
		//ROS_INFO(" GO:%s  steering = %f\n",eta<0?"Right":"Left",steering);		
        //ROS_INFO("Vcmd = %f\tv = %f\tu = %f\n", Vcmd, v, u);

		msg.data.push_back(steering);
		msg.data.push_back(u);

		pub_.publish(msg);
        ROS_INFO("steering = %f\tu = %f",msg.data[0],msg.data[1]);
	};

private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  ros::Subscriber waypoint_sub;
  ros::Publisher pub_;
  
  geometry_msgs::PoseWithCovarianceStamped mypose_now;
  geometry_msgs::PoseWithCovarianceStamped mypose_last;

  nav_msgs::Path path;
  
  double velocity_x, velocity_y;
  double roll_now, pitch_now, yaw_now;
  double roll_last, pitch_last, yaw_last;

  ros::Timer timer1;
  ros::Time current_time;
  ros::Time last_time;
  const double L;
  const double Lfw;
  const double Vcmd;
  const double lfw;
  const double Kp;
  double steering;
  double eta;
  double u; 

  int reached_tag;

};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pose_listener");

  //Create an object of class SubscribeAndPublish that will take care of everything
  PoseSubscriiber pose_listener;

  ros::spin();

  return 0;
}
