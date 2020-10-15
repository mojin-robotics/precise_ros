#pragma once

#include <iostream>
#include <vector>
#include <mutex>
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

    struct Response
    {
        std::string message;
    };

    class TCPConnection
    {
    public:
        TCPConnection(){}
        ~TCPConnection(){}

        bool connect()
        {
            std::cout<<"connecting..."<<std::endl;
            std::cout<<"connected"<<std::endl;
        }
        Response sendCommand(std::string cmd)
        {
            cmd.append("\n");
            std::cout<<"sending: "<<cmd<<std::endl;
            Response res;
            res.message="succeeded";
        }

    };

    class PFlexDevice
    {
    public:
        PFlexDevice(TCPConnection &connection);
        ~PFlexDevice();

        bool init(int profile_no, Profile profile);

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

        bool setHp(bool enabled);
        bool getHp();
        bool waitForEom(double timeout);

        std::vector<double> getJointPositions();
        geometry_msgs::Pose getCartesianPosition();

        bool moveCartesian(geometry_msgs::Pose pose);
        bool moveJointSpace(std::vector<double> joints);

        bool freeMode(bool enabled);

    private:
        TCPConnection* connection_;
        bool is_selected_;
        bool is_attached_;
        bool is_hp_;
        bool is_homed_;

        std::string ip_address_;
        int port_;
        
        std::mutex comm_mutex;
    };

}