// transform the origin odom to a base frame (usually imu to map)

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

constexpr int LISTEN_DURATION{10};
constexpr int PUBLISH_RATE{25};

class TransformWorldOdom {
    private:
        ros::Subscriber sub_origin_odom;
        ros::Publisher pub_world_odom;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        std::string node_name;
        std::string target_frame;
    public:
        
        TransformWorldOdom(std::string name, ros::NodeHandle &nh) :
        node_name(std::move(name)), tf_listener(tf_buffer)
        {
            target_frame = uwrt_mars_rover_utils::getParam<std::string>(
                nh, node_name, "target_frame", "map");

            std::string odom_topic = uwrt_mars_rover_utils::getParam<std::string>(
                nh, node_name, "odom_topic", "vectornav/Odom");
            
            std::string transform_topic = target_frame + "/Odom";
            
            sub_origin_odom = nh.subscribe(odom_topic, LISTEN_DURATION, &TransformWorldOdom::transformOdom, this);
            pub_world_odom = nh.advertise<nav_msgs::Odometry>(transform_topic, PUBLISH_RATE);
        }

        void transformOdom(const nav_msgs::OdometryConstPtr & odom){
            nav_msgs::Odometry world_odom;

            geometry_msgs::PoseStamped origin_pose;
            geometry_msgs::PoseStamped target_pose;

            geometry_msgs::Twist origin_twist;
            geometry_msgs::Twist target_twist;

            try{
                // transform the pose from base frame to the target frame
                origin_pose.pose = odom->pose.pose;
                origin_pose.header = odom->header;

                tf_buffer.transform(origin_pose, target_pose, target_frame);

                world_odom.pose.pose = target_pose.pose;
                world_odom.pose.covariance = odom->pose.covariance;

                // transform the twist from base frame to the target frame
                origin_twist = odom->twist.twist;
                
                transformTwist(origin_twist, target_twist, odom->header.frame_id);

                world_odom.twist.twist = target_twist;
                world_odom.twist.covariance = odom->twist.covariance;

                // populate header and frame information
                world_odom.header.frame_id = target_frame;
                world_odom.child_frame_id = target_frame;

                world_odom.header.stamp = ros::Time::now();

                // publish transformed odometry
                pub_world_odom.publish(world_odom);
            }
            catch (tf2::TransformException &ex) {
                ROS_ERROR_STREAM_NAMED(node_name, "Received an exception trying to transform pose from " << odom->header.frame_id <<
                                        " to " << target_frame <<  ": " << ex.what());
            }
        }

        void transformTwist(const geometry_msgs::Twist & origin_twist, geometry_msgs::Twist & target_twist, const std::string &origin_frame) {
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped = tf_buffer.lookupTransform(target_frame, origin_frame, ros::Time(0));

            transform_stamped.transform.translation.x = 0;
            transform_stamped.transform.translation.y = 0;
            transform_stamped.transform.translation.z = 0;

            tf2::doTransform(origin_twist.linear, target_twist.linear, transform_stamped);
            tf2::doTransform(origin_twist.angular, target_twist.angular, transform_stamped);
        }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "publishWorldOdom");
  ros::NodeHandle nh;

  TransformWorldOdom transform_odom("publishWorldOdom", nh);

  ros::spin();

  return 0;
}