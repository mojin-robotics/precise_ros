#pragma once

#include <precise_driver/device/tcp_client.h>
#include <precise_driver/device/queue.h>

#include <iostream>
#include <vector>
#include <mutex>
#include <thread>

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

    class Device
    {
    public:
        //contruct driver
        explicit Device(std::shared_ptr<TCPClient> connection,
                    std::shared_ptr<TCPClient> status_connection);
        //destruct driver
        ~Device();


        //basic commands
        //====================================

        //try connect to manipulator and powers it up
        bool init(const int profile_no, const Profile profile);

        //home proceduce of the manipulator
        bool home();

        //halts the manipulator
        bool halt();

        //recover the manipulor after an error (TODO: implement)
        bool recover();

        //sends a no operation command to manipulator (test send/response)
        bool nop();

        //closes the connection from manipulator side
        bool exit();

        //reads the robot state from the communication thread
        bool updateRobotState();

        //enables actuatur power
        bool setHp(const bool enabled, const int timeout=0);
        //get state of manipulator power up
        bool getHp();

        //get manipulators SystemState as an enum (TODO: implement enum)
        int getSysState(const bool mute);

        //selects the manipulator to control (in case multiple manipulators should be controlled over the the same connection (daisy chain))
        bool selectRobot(const int robot=-1);

        //attach this communication instance to the robot
        bool attach(const bool flag);

        //enable/disable zero gravity composition (teach mode)
        bool freeMode(const bool enabled);

        //check if manipulator is operational for commanding joint states
        bool operational();

        //gets the manipulators actual mode (TODO: define description mode -> enum)
        int getMode();
        //set the manipulatos mode (TODO: define description mode -> enum)
        bool setMode(const int mode);


        //configuration commands
        //====================================

        //set configuration for a specific motion profile
        bool setProfile(const int profile_no, const Profile& profile);
        //get configuration of a specific motion profile
        Profile getProfile(const int profile_no);

        //sets the manipulators internal base coordination system (used for moveCartesianPositon commands)
        bool setBase(const geometry_msgs::Pose& pose);
        //gets the manipulators internal base coordination system (used for moveCartesianPositon commands)
        geometry_msgs::Pose getBase();

        //sets Playload the manipulator should deal with (force control)
        bool setPayload(const int payload);
        //gets Playload the manipulator is configured to
        int getPayload();

        //sets the max speed for the selected profile. Speed is set in percentage of the max posibile
        //speed of the manipulator. (0=0% of max velocity, 100=100% of max velocity)
        bool setSpeed(const int profile_no, const int speed);
        //gets the configured speed(velocity) of a certain profile
        int getSpeed(const int profile_no);


        //motion related methos
        //====================================

        //gets the actual joint states
        std::vector<double> getJointPositions();
        //move the manipulator to a position in joint space
        bool moveJointPosition(const int profile_no, const std::vector<double>& joints);
        //queues a joint space position to the command queue
        bool queueJointPosition(const int profile_no, const std::vector<double>& joints);

        //gets the manipulators cartesian position of the end effector
        //relative to the configured base position (see setBase,getBase)
        geometry_msgs::Pose getCartesianPosition();
        //moves to manipulator to the given cartesian position
        //relative to the configured base position (see setBase,getBase)
        bool moveCartesianPosition(const int profile_no, const geometry_msgs::Pose& pose);

        //blocks until the current executing motion is finished
        bool waitForEom();

        //custom commands
        //====================================

        //custom command, normally not supplied by defaul manipulator (robot) hardware/software
        //force supported grasp of an object. (supports a forced feedback. It's depending on the
        //parameters given values.
        bool graspPlate(const int width, const int speed, const double force);
        //custom command, normally not supplied by defaul manipulator (robot) hardware/software
        //releases the gripped object. Opens till the width is reached
        bool releasePlate(const int width, const int speed);

        //send a custom command to the manipulator (any command that the manipulator supports, except motion commands)
        std::string command(const std::string& cmd);


        //command queue commands (sould get just internal)
        //====================================

        // thread safe command queue methods

        //start internal driver communication queue update thread
        void startCommandThread();

        //clear internal update queue
        void clearCommandQueue();

    private:
        std::shared_ptr<TCPClient> connection_;
        std::shared_ptr<TCPClient> status_connection_;

        bool is_attached_;
        bool is_hp_;
        bool is_homed_;
        bool is_teachmode_;
        int sys_state_;

        std::mutex mutex_state_data_;

        std::vector<double> transform_vec_ = {0.001, M_PI / 180, M_PI / 180, M_PI / 180, 0.0005};

        Queue<std::string> command_queue_;
        std::thread command_thread_;

        //internal command update funktion
        void updateCommand();
    };
}