#include <ros/common.h>
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <precise_driver/precise_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "precise_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  #if ROS_VERSION_MINIMUM(1, 15, 0) // ros noetic is 1.15.x
    std::shared_ptr<precise_driver::PreciseHWInterface> precise_hw_interface
      (new precise_driver::PreciseHWInterface(nh));
  #else
    boost::shared_ptr<precise_driver::PreciseHWInterface> precise_hw_interface
      (new precise_driver::PreciseHWInterface(nh));
  #endif

  precise_hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, precise_hw_interface);
  control_loop.run(); // Blocks until shutdown signal recieved

  return 0;
}