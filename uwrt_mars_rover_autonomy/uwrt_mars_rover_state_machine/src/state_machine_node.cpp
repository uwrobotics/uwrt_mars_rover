
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <uwrt_mars_rover_spiral/spiralSearchAction.h>
#include <uwrt_mars_rover_drive_to_ar/driveToArAction.h>
#include "uwrt_mars_rover_state_machine/state_machine.h"
#include <uwrt_mars_rover_msgs/set_state.h>

#include <cmath>
#include <vector>

// TODO: grab these from the parameter server
constexpr int LISTEN_DURATION{10};
ros::Duration COMMAND_TIMEOUT(15);

class StateMachine {
  private:
    enum {
      STATE_INIT = 0,
      STATE_WAIT_FOR_GPS,
      STATE_DRIVE_TO_GPS,
      STATE_INIT_SPIRAL_SEARCH,
      STATE_SEND_GOAL_SPIRAL_SEARCH,
      STATE_SPIRAL_SEARCH_AND_DETECT,
      STATE_AR_TAGS_DETECTED,
      STATE_DRIVE_TO_AR_TAGS,
      STATE_FLASH_LEDS,
      STATE_SUCCESS,
      STATE_FAILURE,
      STATE_BACKTRACK,
      STATE_WATSON
    } eState;

    ros::Subscriber ar_tracker;

    ros::ServiceServer service_;
    ros::ServiceClient neopixel_client;
    
    uwrt_mars_rover_spiral::spiralSearchGoal spiral_goal;
    uwrt_mars_rover_drive_to_ar::driveToArGoal ar_goal;

    uwrt_mars_rover_msgs::set_state srv;

    actionlib::SimpleActionClient<uwrt_mars_rover_spiral::spiralSearchAction> spiral_client;
    actionlib::SimpleActionClient<uwrt_mars_rover_drive_to_ar::driveToArAction> ar_client;

    int ar_count;
    int backtrack_count;
    int ar_num_detected;
    bool start_server;
    bool spiral_finished;
    bool ar_finished;
    bool ar_update;

    std::string node_name;

    ros::Time command_time;



  public:

    StateMachine(std::string name, ros::NodeHandle & nh) : 
      node_name(std::move(name)),
      eState(STATE_INIT),
      ar_count(0),
      spiral_client("spiralSearch", true),
      ar_client("driveToAr", true),
      start_server(false),
      spiral_finished(false),
      ar_finished(false),
      backtrack_count(0),
      command_time(0),
      ar_update(false),
      ar_num_detected(0)
    {
      ar_tracker = nh.subscribe("/ar_pose_marker", LISTEN_DURATION, &StateMachine::ar_detection, this);
      service_ = nh.advertiseService("state_machine", &StateMachine::setState, this);
      neopixel_client = nh.serviceClient<uwrt_mars_rover_msgs::set_state>("neopixel_set");
    }

    void run() {

      switch(eState)
      {
        case STATE_INIT:
        {
          ar_count = 0;
	        ar_num_detected = 0;
          ar_goal.markers.clear();
          ar_goal.num_of_tags = 0;

          spiral_goal.radius = 0;
          spiral_goal.angular_velocity = 0;
          spiral_goal.spiral_constant = 0;

          spiral_finished = false;
          ar_finished = false;

          srv.request.requested_mode.value = 0;
	        ros::Duration(1).sleep();
          if (!neopixel_client.call(srv)) {
            backtrack_count = 3;
            ROS_WARN_STREAM_NAMED(node_name, "FAILED TO SET NEOPIXEL STATE");
            eState = STATE_FAILURE;
          }

          if (spiral_client.isServerConnected() && ar_client.isServerConnected()) {
            ROS_INFO_STREAM_NAMED(node_name, "Spiral and AR servers started!");
            ROS_INFO_STREAM_NAMED(node_name, "\n\n\n ********Autonomy State Machine Started*********\n");
            eState = STATE_INIT_SPIRAL_SEARCH;
          }
          break;
        }
        case STATE_INIT_SPIRAL_SEARCH:
        {
          spiral_goal.radius = 2.2;
          spiral_goal.angular_velocity = 1.2;
          spiral_goal.spiral_constant = 0.04;
          if (start_server) {
            ROS_INFO_STREAM_NAMED(node_name, "SENDING SPIRAL GOAL");
            eState = STATE_SEND_GOAL_SPIRAL_SEARCH;
          }
          break;
        }
        case STATE_SEND_GOAL_SPIRAL_SEARCH:
        {
          ar_num_detected = 0;
          ar_goal.markers.clear();
          ar_goal.num_of_tags = 0;
          command_time = ros::Time::now();
          spiral_client.sendGoal(spiral_goal,
                boost::bind(&StateMachine::spiral_server_callback, this, _1, _2),
                actionlib::SimpleActionClient<uwrt_mars_rover_spiral::spiralSearchAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<uwrt_mars_rover_spiral::spiralSearchAction>::SimpleFeedbackCallback());
	        ROS_INFO_STREAM_NAMED(node_name, "ATTEMPTING TO DETECT AR TAGS");
          eState = STATE_SPIRAL_SEARCH_AND_DETECT;
          break;
        }
        case STATE_SPIRAL_SEARCH_AND_DETECT:
        {
	        ros::Time time_now = ros::Time::now();
          // check ar tags
          if (ar_count > 2) {
            command_time = ros::Time::now();
            spiral_client.cancelAllGoals();
	          ar_count = 0;
	          eState = STATE_AR_TAGS_DETECTED;
            ros::Duration(1).sleep();
          }

          if (spiral_finished || (time_now - command_time) > COMMAND_TIMEOUT) {
            ROS_WARN_STREAM_NAMED(node_name, "FAILED TO DETECT AR TAGS, eState = STATE_SPIRAL_SEARCH");
            eState = STATE_FAILURE;
          }
          break;
        }
	      case STATE_AR_TAGS_DETECTED:
	      {
          ros::Time time_now = ros::Time::now();

          if ((time_now - command_time) > COMMAND_TIMEOUT) {
            eState = STATE_SEND_GOAL_SPIRAL_SEARCH;
          }

          else if (ar_goal.num_of_tags > 0) {
            ROS_INFO_STREAM_NAMED(node_name, "DRIVING TO AR TAGS");
            command_time = ros::Time::now();
            ar_client.sendGoal(ar_goal,
                  boost::bind(&StateMachine::ar_server_callback, this, _1, _2),
                  actionlib::SimpleActionClient<uwrt_mars_rover_drive_to_ar::driveToArAction>::SimpleActiveCallback(),
                  actionlib::SimpleActionClient<uwrt_mars_rover_drive_to_ar::driveToArAction>::SimpleFeedbackCallback());
            eState = STATE_DRIVE_TO_AR_TAGS;
          }
          break;
	      }
        case STATE_DRIVE_TO_AR_TAGS:
        {
      	  ros::Time time_now = ros::Time::now();
          if (ar_finished) {
            eState = STATE_FLASH_LEDS;
          }
          // we should be checking if we detected another ar tag or if we are stuck or if we are off track somehow
          if (ar_update) {
            ar_client.cancelAllGoals();
            // clear ar data
            ar_num_detected = 0;
            ar_goal.markers.clear();
            ar_goal.num_of_tags = 0;
            eState = STATE_AR_TAGS_DETECTED;
          }
          else if ((time_now - command_time) > COMMAND_TIMEOUT)
          {
            ar_client.cancelAllGoals();
            eState = STATE_FAILURE;
          }
          break;
        }
        case STATE_FLASH_LEDS:
        {
          if (flash_leds(10)) {
            eState = STATE_SUCCESS;
      	  }
          else {
            ROS_WARN_STREAM_NAMED(node_name, "FAILED TO CALL NEOPIXEL SERVER, eState = STATE_FLASH_LEDS");
            eState = STATE_FAILURE;
          }
          break;
        }
        case STATE_SUCCESS:
        {
          ROS_INFO_STREAM_NAMED(node_name, "SUCCESSFULLY REACHED GOAL AUTONOMOUSLY");
          eState = STATE_WATSON;
          break;
        }
        case STATE_FAILURE:
        {
          // backtrack here
          ROS_WARN_STREAM_NAMED(node_name, "FAILED TO REACH GOAL AUTONOMOUSLY.......");
          if (backtrack_count < 3)
          {
            ROS_WARN_STREAM_NAMED(node_name, "RETRYING ATTEMPT " << backtrack_count << ".......");
            eState = STATE_BACKTRACK;
          }
          break;
        }
        case STATE_BACKTRACK:
        {
          // try to detect ar tags
          // try to spiral bigger?
          // spin gimbal to detect ar tags
          // for now just see if we have any ar tags
          ar_count = 0;
	        ar_num_detected = 0;
          ar_goal.markers.clear();
          ar_goal.num_of_tags = 0;
          spiral_finished = false;
          ar_update = false;
          command_time = ros::Time::now();
          backtrack_count++;
          // set cmd vel 0
          eState = STATE_SPIRAL_SEARCH_AND_DETECT;
          break;
        }
        case STATE_WATSON:
        {
          start_server = false;
          eState = STATE_INIT;
          break;
        }
        default:
          break;
      }
    }

    bool flash_leds(double time) {
      ros::Time time_now = ros::Time::now();
      ros::Time time_curr = ros::Time::now();
      ros::Duration d(time);
      double value = 2;

      ros::Rate loop{2};

      while ((time_curr - time_now) < d && ros::ok()) {
        srv.request.requested_mode.value = value;
        if (!neopixel_client.call(srv)) {
          ROS_WARN_STREAM_NAMED(node_name, "Failed to send LED command " << value);
          return false;
        }

        if (value == 2) {
          value = 1;
        } else {
          value = 2;
        }
        loop.sleep();
        time_curr = ros::Time::now();
      }

      return true;
    }

    void ar_detection(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ar_tags) {
      // sanity check for confidence
      int size = ar_tags->markers.size();

      // detected different number of tags so lets backtrack and determine what to do
      if (size != ar_goal.num_of_tags && ar_num_detected == 10) {
        // this could be cause we approached the pole or gate and its not in vision anymore
        // trust that we saw the tag and just go with it
        if (size != 0)
        {
          ar_finished = false;
          ar_update = true;
        }
      }

      // capture 2 events before we say that we have found tags
      if (size > 0 && ar_count < 3) {
        ar_count = ar_count + 1;
      }
	
      if (size == 1) {
	      ar_num_detected++;
      }


      if (ar_num_detected == 10) {
        for (int i = 0; i < ar_tags->markers.size(); i++) {
          ar_goal.markers.push_back(ar_tags->markers[i].pose.pose);
        }
        ar_goal.num_of_tags = ar_goal.markers.size();
      }
    }

    void spiral_server_callback(const actionlib::SimpleClientGoalState & state, const uwrt_mars_rover_spiral::spiralSearchResult::ConstPtr & spiral_result) {
      if (spiral_result->spiral_complete) {
        // did not detect ar tags 
        spiral_finished = true;
      }
    }

    void ar_server_callback(const actionlib::SimpleClientGoalState & state, const uwrt_mars_rover_drive_to_ar::driveToArResult::ConstPtr & ar_result) {
      // are we within 3 m of the ar tag
      if (ar_result->pos_to_goal < 3) {
        ar_finished = true;
      }
    }

    bool setState(uwrt_mars_rover_state_machine::state_machine::Request &req, uwrt_mars_rover_state_machine::state_machine::Response &res)
    {
      if (req.requested_mode == 1) {
        start_server = true;
      }
      res.success = true;
      return true;
    }
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;

  StateMachine state_machine("state_machine", nh);

  ros::Rate loop_rate{25};

  while (ros::ok())
  {
    state_machine.run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
