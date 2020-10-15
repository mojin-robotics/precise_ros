#pragma once

#include <iostream>
#include <vector>
#include <geometry_msgs/Pose.h>

namespace precise_driver
{
    struct Profile{
        double speed;
        double speed2;
        double accel;
        double decel;
        double accel_ramp;
        double decel_ramp;
        int in_range;
        int straight;
    };

    class PFlexDevice
    {
    public:
        PFlexDevice(/*TCPConnection &connection*/);
        ~PFlexDevice();

        bool init(Profile profile);

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

        bool setSpeed(double speed);
        bool getSpeed(double speed);

        bool setHp(bool enabled);
        bool getHp();
        bool waitForEom(double timeout);

        std::vector<double> getJointPositions();
        geometry_msgs::Pose getCartesianPosition();

        bool moveCartesian(geometry_msgs::Pose pose);
        bool moveJointSpace(std::vector<double> joints);

        bool freeMode(bool enabled);

    private:
        //TCPConnection* connection_;
        bool is_selected_;
        bool is_attached_;
        bool is_hp_;
        bool is_homed_;

        std::string ip_address_;
        int port_;
    };

}