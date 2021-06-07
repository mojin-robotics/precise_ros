#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <precise_driver/device/device.h>
#include <precise_driver/device/tcp_client.h>
#include <precise_driver/Gripper.h>
#include <precise_driver/Plate.h>
#include <ros/ros.h>

#include <mutex>
#include <condition_variable>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <cob_srvs/SetString.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>

namespace precise_driver
{

    class PreciseHWInterface : public ros_control_boilerplate::GenericHWInterface
    {
    public:
        PreciseHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
        ~PreciseHWInterface();

        virtual void init();
        virtual void read(ros::Duration &elapsed_time);
        virtual void write(ros::Duration &elapsed_time);
        virtual void enforceLimits(ros::Duration &period);

    private:
        ros::ServiceServer init_srv_;
        ros::ServiceServer recover_srv_;
        ros::ServiceServer teachmode_srv_;
        ros::ServiceServer teachmode_gripped_srv_;
        ros::ServiceServer home_srv_;
        ros::ServiceServer power_srv_;
        ros::ServiceServer attach_srv_;
        ros::ServiceServer cmd_srv_;
        ros::ServiceServer grasp_plate_srv_;
        ros::ServiceServer release_plate_srv_;
        ros::ServiceServer gripper_srv_;

        ros::ServiceClient switch_controller_srv_;

        std::shared_ptr<Device> device_;
        Profile profile_;
        int profile_no_;

        std::mutex mutex_init_;
        std::mutex mutex_write_;
        std::condition_variable cond_init_;

        bool write_enabled_;

        void enableWrite(bool value);
        bool isWriteEnabled();

        bool resetController(bool active);

        bool initCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool recoverCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool teachmodeCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool teachmodeGrippedCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool homeCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool powerCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool attachCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool cmdCb(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res);

        bool gripperCB(precise_driver::Gripper::Request &req, precise_driver::Gripper::Response &res);

        bool graspPlateCB(precise_driver::Plate::Request &req, precise_driver::Plate::Response &res);
        bool releasePlateCB(precise_driver::Plate::Request &req, precise_driver::Plate::Response &res);

    //Doosan like Hack
    private:
        bool doosan_hack_enabled_;
        ros::Subscriber sub_follow_joint_goal;
        void followJointTrajectoryActionGoalCB(const control_msgs::FollowJointTrajectoryActionGoalConstPtr &msg);

    }; // class

} // namespace precise_driver
