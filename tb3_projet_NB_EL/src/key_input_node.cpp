#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "termios.h"

using namespace ros;

int getch();
std_msgs::Int8 keyboard_input;
Publisher KeyInput_Pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "key_input_node");

  ros::NodeHandle nh;

  // Publishers
  KeyInput_Pub = nh.advertise<std_msgs::Int8>("/keyboard_input", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    keyboard_input.data = getch();
    KeyInput_Pub.publish(keyboard_input);
    ROS_INFO("keyb = %d", keyboard_input.data);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int getch() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);               // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}
