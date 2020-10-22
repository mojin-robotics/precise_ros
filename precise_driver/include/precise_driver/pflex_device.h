#pragma once

#include <precise_driver/precise_tcp_interface.h>

#include <iostream>
#include <vector>
#include <mutex>

#include <geometry_msgs/Pose.h>

namespace precise_driver
{
    struct Profile{
        int speed;
        int speed2;
        int accel;
        int decel;
        double accel_ramp;
        double decel_ramp;
        int in_range;
        int straight;
    };

    class PFlexDevice
    {
    public:
        explicit PFlexDevice(std::shared_ptr<PreciseTCPInterface> connection);
        ~PFlexDevice();

        bool init(int profile_no, Profile profile);
        bool exit();
        bool halt();
        bool home();
        bool attach(bool flag);
        bool selectRobot(int robot=-1);
        bool setBase(geometry_msgs::Pose pose);
        geometry_msgs::Pose getBase();
        Profile getProfile(int profile_no);
        bool setProfile(int profile_no, Profile profile);
        bool nop();
        bool setPayload(int payload);
        int getPayload();
        bool setSpeed(int speed);
        int getSpeed();
        bool setHp(bool enabled, int timeout=0);
        bool getHp();
        bool waitForEom(double timeout);
        std::vector<double> getJointPositions();
        geometry_msgs::Pose getCartesianPosition();
        bool moveCartesian(int profile_no, geometry_msgs::Pose pose);
        bool moveJointSpace(int profile_no, std::vector<double> joints);
        bool freeMode(bool enabled);
        int getSysState(bool mute);
        int getMode();
        bool setMode(int mode);
        bool operable();

    private:
        std::shared_ptr<PreciseTCPInterface> connection_;
        bool is_attached_;
        bool is_hp_;
        bool is_homed_;
        bool is_teachmode_;

        std::mutex mutex_state_data_;

        std::vector<double> transform_vec_ = {0.001, M_PI / 180, M_PI / 180, M_PI / 180, 0.0005};
    };

}