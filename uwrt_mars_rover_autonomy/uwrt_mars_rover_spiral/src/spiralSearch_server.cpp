/*
Action server for the spiral search algorithm
Currently works with turtlesim for testing
Spiral constant default = 0.25 , Angular velocity default = 1m/s
*/

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <uwrt_mars_rover_spiral/spiralSearchAction.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

#include <cmath>

// TODO: grab these from the parameter server
constexpr int MAX_SPIRAL_RADIUS{10};
constexpr int MIN_SPIRAL_RADIUS{1};
constexpr int LOOP_RATE{50};
constexpr int PUBLISH_RATE{1};

// TODO: grab the max linear speed from parameter server
constexpr double MAX_LINEAR_SPEED{5};
constexpr double MAX_ANGULAR_VEL{3};
constexpr double MAX_SPIRAL_CONSTANT{0.25};

class SpiralSearch {
 private:
  uwrt_mars_rover_spiral::spiralSearchFeedback feedback;
  uwrt_mars_rover_spiral::spiralSearchResult result;

  ros::Publisher velocity_publisher;

  actionlib::SimpleActionServer<uwrt_mars_rover_spiral::spiralSearchAction> server;

  std::string node_name;
  std::string cmd_vel;

 public:
  SpiralSearch(std::string name, ros::NodeHandle &nh)
      : node_name(std::move(name)), server(nh, "spiralSearch", boost::bind(&SpiralSearch::execute, this, _1), false) {
    std::string cmd_vel = uwrt_mars_rover_utils::getParam<std::string>(
        nh, node_name, "cmd_vel", "/uwrt_mars_rover/drivetrain_velocity_controller/cmd_vel");

    velocity_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel, PUBLISH_RATE);
  }

  void execute(const uwrt_mars_rover_spiral::spiralSearchGoalConstPtr &goal) {
    if (goal->radius < MIN_SPIRAL_RADIUS || goal->radius > MAX_SPIRAL_RADIUS) {
      ROS_WARN_STREAM_NAMED(node_name, "Please set radius to a valid value between " << MIN_SPIRAL_RADIUS << "m and "
                                                                                     << MAX_SPIRAL_RADIUS << "m.");
    } else if (goal->angular_velocity < 0 || goal->spiral_constant < 0) {
      ROS_WARN_STREAM_NAMED(node_name, "Please set spiral constant and angular velocity above 0");
    } else {
      double angular_velocity = goal->angular_velocity;
      double linear_velocity = 0.0;
      double spiral_constant = goal->spiral_constant;

      if (angular_velocity > MAX_ANGULAR_VEL) {
        ROS_INFO_STREAM_NAMED(node_name, "Clamped max angular velocity to " << MAX_ANGULAR_VEL);
        angular_velocity = MAX_ANGULAR_VEL;
      }

      if (goal->spiral_constant > MAX_SPIRAL_CONSTANT) {
        ROS_INFO_STREAM_NAMED(node_name, "Clamped max spiral constant to " << MAX_SPIRAL_CONSTANT);
        spiral_constant = MAX_SPIRAL_CONSTANT;
      }

      // change angular velocity to ensure goal is met
      while (angular_velocity * goal->radius > MAX_LINEAR_SPEED) {
        angular_velocity /= 2;
      }

      geometry_msgs::Twist vel_msg;

      // this is the frequency that the loop will follow, by keeping track of "sleep()"
      ros::Rate loop_rate(LOOP_RATE);

      double start_time = ros::Time::now().toSec();
      double prev_time = 0.0;
      double current_time = 0.0;
      double time_delta = 0.0;

      double radius = 0.0;
      double final_circle_dist = 0.0;
      double circumference = goal->radius * 2 * M_PI;

      bool success = false;

      vel_msg.linear.x = linear_velocity;
      vel_msg.angular.z = angular_velocity;

      // Run until we have spiraled for a predefined radius or until our linear speed is too high (FAILSAFE)
      while (final_circle_dist < circumference && vel_msg.linear.x < MAX_LINEAR_SPEED) {
        if (server.isPreemptRequested() || !ros::ok()) {
          server.setPreempted();
          success = false;
          break;
        }

        current_time = ros::Time::now().toSec();

        if (radius < goal->radius) {
          time_delta = current_time - start_time;
          linear_velocity = spiral_constant * time_delta;

          // TODO: change this to use encoders and vectornav to accurately determine the radius
          radius = linear_velocity / angular_velocity;
          feedback.current_radius = radius;

          prev_time = ros::Time::now().toSec();
        } else {
          time_delta = current_time - prev_time;
          // TODO: change this to use encoders and vectornav to accurately determine the distance
          final_circle_dist = angular_velocity * radius * time_delta;
        }

        vel_msg.linear.x = linear_velocity;
        velocity_publisher.publish(vel_msg);

        server.publishFeedback(feedback);

        ros::spinOnce();

        loop_rate.sleep();
      }

      result.vel_linear = vel_msg.linear.x;

      // this stops the turtle
      vel_msg.linear.x = 0;
      vel_msg.angular.z = 0;
      velocity_publisher.publish(vel_msg);

      if (final_circle_dist >= circumference) {
        server.setSucceeded(result);
      } else if (linear_velocity >= MAX_LINEAR_SPEED) {
        ROS_WARN_STREAM_NAMED(node_name,
                              "Reached max linear speed so unable to complete spiral, try decreasing spiral constant");
        server.setAborted(result);
      } else {
        server.setAborted(result);
      }
    }
  }

  void startServer() {
    ROS_INFO_NAMED(node_name, "\n\n\n ********Spiral Search Server Started*********\n");
    server.start();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "spiralSearch");
  ros::NodeHandle nh;

  SpiralSearch spiral("spiralSearch", nh);

  spiral.startServer();

  ros::spin();

  return 0;
}