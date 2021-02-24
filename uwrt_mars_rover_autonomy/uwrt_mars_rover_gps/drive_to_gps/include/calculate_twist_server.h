#pragma once
#include <ros/ros.h>
#include <utils.h>
#include <uwrt_mars_rover_msgs/gps_heading.h>
#include <uwrt_mars_rover_msgs/gps_goalAction.h>
#include <geometry_msgs/Twist.h>

#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>

class TwistAction {

    protected:
        actionlib::SimpleActionServer<uwrt_mars_rover_msgs::gps_goalAction> as;

    public:
        TwistAction(std::string name, ros::NodeHandle n) : 
            as(n, name, boost::bind(&TwistAction::executeCB, this, _1), false),
            node(n){}
        /*
        TwistAction(TwistAction&&) = default;
        TwistAction& operator=(TwistAction&&) = default;
        TwistAction(const TwistAction&) = delete;
        TwistAction& operator=(const TwistAction&) = delete; */
        ~TwistAction() = default;

        ros::CallbackQueue queue_;
        ros::NodeHandle node;

        void init() {
            as.start();
        }

        // action server callback
        void executeCB(const uwrt_mars_rover_msgs::gps_goalGoalConstPtr &goal) {
            uwrt_mars_rover_msgs::gps_goalFeedback feedback;
            if (curr_gps == NULL || curr_head == NULL) {
                ros::Rate queue_sleep(1);
                ROS_INFO("Waiting for topics to publish data");

                while (ros::ok() && (curr_gps == NULL || curr_head == NULL)) {
                    queue_.callAvailable();
                    queue_sleep.sleep();
                }
            }

            sensor_msgs::NavSatFixPtr gps_goal(new sensor_msgs::NavSatFix(goal->gps_goal));
            bool end = false;
            ros::Rate rate(1);

            while (calculate_distance(gps_goal, curr_gps) > MAX_ERROR && !end && ros::ok()) {
                ROS_INFO_STREAM("Calculating degrees, heading and sending twist message");

                geometry_msgs::Twist msg;
                int goal_heading = calculate_degrees(gps_goal, curr_gps);
                int multiplier = goal_heading > curr_head->degrees ? 1 : -1;
                int heading_diff = abs(goal_heading - curr_head->degrees);

                ROS_INFO_STREAM("Heading between current heading and goal is " << heading_diff << " and direction is " << multiplier);

                msg.angular.z = heading_diff > 4 ? multiplier * MAX_ANGULAR_VEL : 0;
                msg.linear.x = MAX_LINEAR_VEL;
                feedback.go_to_goal = msg;

                as.publishFeedback(feedback);
                queue_.callAvailable();
                rate.sleep();
            }
            uwrt_mars_rover_msgs::gps_goalResult r;

            r.arrived = !end;
            as.setSucceeded(r);
        }

        void update_current_heading(const uwrt_mars_rover_msgs::gps_headingConstPtr &c_head) {
            if (curr_head == NULL) 
                curr_head = uwrt_mars_rover_msgs::gps_headingPtr(new uwrt_mars_rover_msgs::gps_heading);
            curr_head->degrees = c_head->degrees;
        }

        void update_current_gps(const sensor_msgs::NavSatFixConstPtr &gps_current) {
            if (curr_gps == NULL)
                curr_gps = sensor_msgs::NavSatFixPtr(new sensor_msgs::NavSatFix);
            curr_gps->latitude = gps_current->latitude;
            curr_gps->longitude = gps_current->longitude;
        }

    private:
        int degrees_to_target = 0;
        const int MAX_ERROR = 5;
        const double MAX_LINEAR_VEL = 1.0;
        const double MAX_ANGULAR_VEL = 1.0;
        uwrt_mars_rover_msgs::gps_headingPtr curr_head = NULL;  // current heading of the rover
        sensor_msgs::NavSatFixPtr curr_gps = NULL;  // current gps coordinates

};
