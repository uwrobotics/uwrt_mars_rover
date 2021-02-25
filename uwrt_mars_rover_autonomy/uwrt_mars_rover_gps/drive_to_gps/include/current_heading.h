#pragma once
#include <ros/ros.h>
#include <utils.h>
#include <drive_to_gps/CurrentHeadingConfig.h>
#include <uwrt_mars_rover_msgs/gps_heading.h>
#include <sensor_msgs/NavSatFix.h>
#include <dynamic_reconfigure/server.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

#include <math.h>
#include <stack>

class CurrentHeading {
    public:
        CurrentHeading(ros::Publisher &pub, std::stack<sensor_msgs::NavSatFixPtr> &stack) : 
                        pub_curr_head(pub), gps_stack(stack) {}
        ~CurrentHeading() = default;

        void add_gps_to_queue(const sensor_msgs::NavSatFixConstPtr &curr_gps) {
            if (std::isnan(curr_gps->latitude) || std::isnan(curr_gps->longitude)) 
                return;          
            sensor_msgs::NavSatFixPtr item /*(new sensor_msgs::NavSatFix())*/;
            if (gps_stack.size() > 0) {
                item->latitude = ewma(curr_gps->latitude, gps_stack.top()->latitude);
                item->longitude = ewma(curr_gps->longitude, gps_stack.top()->longitude);
            } else {
                item->latitude = curr_gps->latitude;
                item->longitude = curr_gps->longitude;
            }
            item->header.stamp = curr_gps->header.stamp;
            ROS_INFO_STREAM("Pushed to queue with lat " << item->latitude << " long " << item->longitude << " and time stamp "
                                              << item->header.stamp);
            gps_stack.push(item);
        }

        void determine_curr_heading(const ros::TimerEvent &event) {
          uwrt_mars_rover_msgs::gps_heading msg;
          ROS_INFO("Called timer callback");
          // check front of queue to make sure we can pop
          if (gps_stack.size() > 0) {
            ROS_INFO("Size queue big");
            bool gps_after_time = false;
            sensor_msgs::NavSatFixPtr curr = gps_stack.top();
            gps_stack.pop();
            sensor_msgs::NavSatFixPtr prev = NULL;
            while (gps_stack.size() > 0) {
              prev = gps_stack.top();
              // ROS_INFO("Went into loop");
              // ROS_INFO_STREAM("Event real " << event.last_real << " temp stamp " << temp->header.stamp);
              if (event.last_real >= prev->header.stamp && !gps_after_time) {
                ROS_INFO_STREAM("Prev lat and long " << prev->latitude << " " << prev->longitude
                                                     << " curr lat and long " << curr->latitude << " "
                                                     << curr->longitude);
                msg.degrees = calculate_degrees(curr, prev);
                ROS_INFO_STREAM("Published " << msg.degrees);
                pub_curr_head.publish(msg);
                gps_after_time = true;
              }
              gps_stack.pop();
            }
            gps_stack.push(curr);
          } 
        }

    private:
        static inline double ewma(double value, double prev_val, double a = 0.5) {
            return value * a + prev_val * a;
        }

    protected:
        ros::Publisher pub_curr_head;
        std::stack<sensor_msgs::NavSatFixPtr> gps_stack;



};