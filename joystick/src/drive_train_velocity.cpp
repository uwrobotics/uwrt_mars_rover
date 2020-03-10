class drive_train_velocity : public drive_train_template{
    
    public:
        int velocity_scalar;
        ros::publisher drive_train_publisher_velocity_;
        void pub_value(geometry_msgs message){
            drive_train_publisher_velocity_.publish(message);
        }
}