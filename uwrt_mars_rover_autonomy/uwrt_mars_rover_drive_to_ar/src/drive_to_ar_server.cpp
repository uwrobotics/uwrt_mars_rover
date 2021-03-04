/*
Action server for the spiral search algorithm
Currently works with turtlesim for testing
Spiral constant default = 0.25 , Angular velocity default = 1m/s
*/

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <uwrt_mars_rover_drive_to_ar/driveToArAction.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

#include <cmath>

// 0.5m within goal is considered accurate enough
constexpr double DISTANCE_TOLERANCE{1.5};
constexpr double MAX_LINEAR_SPEED{5};
constexpr double MAX_ANGULAR_SPEED{2.5};
constexpr int LISTEN_DURATION{50};
constexpr int PUBLISH_RATE{1};
constexpr double DEFAULT_LINEAR_APPROACH{3.0};
constexpr double DEFAULT_ANGULAR_APPROACH{1.5};

// this node is run while we are spiral searching
// search for ar tags
// if we find an ar tag determine how many 1 or 2?
// if 1 drive towards it
// if 2 locate center between closest and furthest one and drive to centerpoint

class DriveToAr {
 private:
  uwrt_mars_rover_drive_to_ar::driveToArFeedback feedback;
  uwrt_mars_rover_drive_to_ar::driveToArResult result;

  ros::Publisher velocity_publisher;
  ros::Subscriber odom_sub;

  ros::Rate rate;

  actionlib::SimpleActionServer<uwrt_mars_rover_drive_to_ar::driveToArAction> server;

  std::string node_name;
  std::string vel_topic;

  bool server_start;

  double init_x;
  double init_y;
  double init_theta;

  double _x;
  double _y;
  double _theta;

  double goal_x;
  double goal_y;

  double last_command_time;

  // higher these are the more aggressive the approach is to the tag
  // range should be between 0 - 10
  double linear_approach;
  double angular_approach;

  bool janky_hack; // uses theoretical kinematics to calculate rover position, not based on any real encoder / IMU data

 public:
  DriveToAr(std::string name, ros::NodeHandle &nh)
      : node_name(std::move(name)),
        server(nh, name, boost::bind(&DriveToAr::execute, this, _1), false),
        server_start(false),
        rate(LISTEN_DURATION),
        init_x(0),
        init_y(0),
        init_theta(0),
        _x(0),
        _y(0),
        _theta(0),
        goal_x(0),
        goal_y(0),
        last_command_time(ros::Time::now().toSec()),
        janky_hack(true) {
    std::string vel_topic = uwrt_mars_rover_utils::getParam<std::string>(
        nh, node_name, "cmd_vel", "/uwrt_mars_rover/drivetrain_velocity_controller/cmd_vel");

    std::string odom_topic = uwrt_mars_rover_utils::getParam<std::string>(nh, node_name, "odom_topic", "/map/Odom");

    linear_approach =
        uwrt_mars_rover_utils::getParam<double>(nh, node_name, "linear_approach", DEFAULT_LINEAR_APPROACH);

    angular_approach =
        uwrt_mars_rover_utils::getParam<double>(nh, node_name, "angular_approach", DEFAULT_ANGULAR_APPROACH);

    odom_sub = nh.subscribe(odom_topic, LISTEN_DURATION, &DriveToAr::updatePose, this);
    velocity_publisher = nh.advertise<geometry_msgs::Twist>(vel_topic, PUBLISH_RATE);
  }

  void execute(const uwrt_mars_rover_drive_to_ar::driveToArGoalConstPtr &goal) {
    // set up our goal pose
    geometry_msgs::Pose goal_pose;
    geometry_msgs::Twist cmd_vel;
    if (goal->num_of_tags == 1) {
      // get the position of this tag and drive to it
      goal_pose = goal->markers[0];
    } else if (goal->num_of_tags == 2) {
      goal_pose.position.x = (goal->markers[0].position.x + goal->markers[1].position.x) / 2;
      goal_pose.position.y = (goal->markers[0].position.y + goal->markers[1].position.y) / 2;
      // TODO: should we consider z positions
      // don't care about quaternion do we??

    } else {
      // should not get here
      ROS_ERROR_STREAM_NAMED(node_name, "Too many tags found! Limit is 2 tags.");
    }
    bool success = true;

    server_start = true;
    goal_x = goal_pose.position.x;
    goal_y = goal_pose.position.y;

    while (euclideanDist() >= DISTANCE_TOLERANCE) {
      if (janky_hack) {
        forceUpdatePose(cmd_vel.linear.x, cmd_vel.angular.z);
      }

      if (server.isPreemptRequested() || !ros::ok()) {
        server.setPreempted();
        success = false;
        break;
      }
      
      cmd_vel.linear.x = setLinearVel(linear_approach);
      cmd_vel.linear.y = 0;
      cmd_vel.linear.z = 0;

      cmd_vel.angular.x = 0;
      cmd_vel.angular.y = 0;
      cmd_vel.angular.z = setAngularVel(angular_approach);

      velocity_publisher.publish(cmd_vel);
      last_command_time = ros::Time::now().toSec();

      feedback.pos_to_goal = euclideanDist();

      server.publishFeedback(feedback);

      ros::spinOnce();
      rate.sleep();
    }

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    velocity_publisher.publish(cmd_vel);

    result.pos_to_goal = euclideanDist();

    if (success) {
      server.setSucceeded(result);
    } else {
      server.setAborted(result);
    }
  }

  void forceUpdatePose(double angular_vel, double linear_vel) {
    if (server_start) {
      _x = 0;
      _y = 0;
      _theta = 0;

      server_start = false;
    }
    double dx, dy, d_theta, time_now;

    time_now = ros::Time::now().toSec();

    d_theta = angular_vel * (time_now - last_command_time);
    _theta += d_theta;

    dx = cos(_theta) * linear_vel * (time_now - last_command_time);
    dy = sin(_theta) * linear_vel * (time_now - last_command_time);

    _x += dx;
    _y += dy;
  }

  void updatePose(const nav_msgs::OdometryConstPtr &odom) {
    geometry_msgs::Pose pose;

    pose = odom->pose.pose;
    tf::Pose tf_pose;
    tf::poseMsgToTF(pose, tf_pose);

    double yaw = tf::getYaw(tf_pose.getRotation());

    if (server_start) {
      init_x = pose.position.x;
      init_y = pose.position.y;

      init_theta = yaw;

      server_start = false;
    }
    _x = pose.position.x - init_x;
    _y = pose.position.y - init_y;

    _theta = yaw - init_theta;
  }

  double euclideanDist() {
    return sqrt(pow((goal_x - _x), 2) + pow((goal_y - _y), 2));
  }

  double euclideanAngle() {
    return atan2(goal_y - _y, goal_x - _x);
  }

  double setAngularVel(double constant) {
    return constant * (euclideanAngle() - _theta);
  }

  double setLinearVel(double constant) {
    return constant * (euclideanDist());
  }

  void startServer() {
    ROS_INFO_NAMED(node_name, "\n\n\n ********Drive to Ar Server Started*********\n");
    server.start();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "driveToAr");
  ros::NodeHandle nh;

  DriveToAr drive_to_ar("driveToAr", nh);

  drive_to_ar.startServer();

  ros::spin();

  return 0;
}