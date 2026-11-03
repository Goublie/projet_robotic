#include <csignal>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/tf.h>

#define FREQUENCY 0.5
#define MAX_LINEAR_SPEED 0.15
#define MAX_ANGULAR_SPEED 2.84
#define LINEAR_SPEED 0.1
#define ANGULAR_SPEED 0.1
#define AV 1
#define ARR 2
int cpt = 0;

enum state { manual, autonomous };

struct position {
  double x;
  double y;
  double angle;
};

using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;

// Les prototypes des fonctions
void sigintHandler(int sig);

// Les variables globales
state current_state = manual;
Publisher pub_msg;
Publisher pub_vel;
Publisher pub_cmd;
Publisher pub_rst;
String msg;
position current_pos;
double linear_speed = LINEAR_SPEED;
double angular_speed = ANGULAR_SPEED;
double frequence = FREQUENCY;
Subscriber KeyInput_Sub;
Subscriber Position_Sub;

// L'impl des fonctions
void avancer(double v) {
  Twist cmd_vel1;
  cmd_vel1.linear.x = v;
  cmd_vel1.angular.z = 0;

  pub_vel.publish(cmd_vel1);
}

void reculer(double v) { avancer(-v); }

void arreter() { avancer(0.0); }

void tourner(double a) {
  Twist cmd_vel2;
  cmd_vel2.linear.x = 0;
  cmd_vel2.angular.z = a;

  pub_vel.publish(cmd_vel2);
}

void send_msg(String msg) {
  ROS_INFO("Publisher : %s", msg.data.c_str());
  pub_msg.publish(msg);
  // a completer...
}

void kbCallback(std_msgs::Int8 kbInput) {
  switch (kbInput.data) {
    // COMMANDES DE VITESSE 1

  case 122: // z
    avancer(0.05);
    break;
  case 115: // s
    reculer(0.05);
    break;
  case 113: // q
    tourner(0.05);
    break;
  case 100: // d
    tourner(-0.05);
    break;
    // COMMANDES DE VITESSE 2

  case 116: // t
    avancer(0.1);
    break;
  case 103: // g
    reculer(0.1);
    break;
  case 102: // f
    tourner(0.1);
    break;
  case 104: // h
    tourner(-0.1);
    break;

    // COMMANDES DE VITESSE 3

  case 105: // i
    avancer(0.15);
    break;
  case 107: // k
    reculer(0.15);
    break;
  case 106: // j
    tourner(0.15);
    break;
  case 108: // l
    tourner(-0.15);
    break;
  case 114: // r to reset
    std_msgs::Empty reset_msg;
    pub_rst.publish(reset_msg);
    break;
  };
}

void move() {
  ROS_INFO("move");

  int mode = AV;
  if (cpt == 2) {
    mode = ARR;
  } else if (cpt == 0) {
    mode = AV;
  } else if (mode == AV) {
    avancer(1);
  } else {
    arreter();
  }

  cpt++;
  cpt %= 2;
}

void test(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pos.x = msg->pose.pose.position.x;
  current_pos.y = msg->pose.pose.position.y;
  current_pos.angle = tf::getYaw(msg->pose.pose.orientation);

  ROS_INFO("Position actuelle : X = %f, Y = %f, theta = %f", current_pos.x,
           current_pos.y, current_pos.angle);
}

int main(int argc, char **argv) {

  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "controleur", ros::init_options::NoSigintHandler);

  NodeHandle nh;
  pub_msg = nh.advertise<String>("/log", 1000);
  pub_cmd = nh.advertise<Twist>("/cmd_vel", 10);
  pub_vel = nh.advertise<Twist>("/cmd_vel", 10);
  pub_rst = nh.advertise<std_msgs::Empty>("/reset", 10);
  KeyInput_Sub = nh.subscribe("/keyboard_input", 1000, kbCallback);
  Position_Sub = nh.subscribe("/odom", 1000, test);

  nh.getParam("user_freq", frequence);

  std::string state_str;
  if (nh.getParam("current_state", state_str)) {
    if (state_str == "autonomous") {
      current_state = autonomous;
    } else {
      current_state = manual;
    }
  }
  // a completer...

  Rate loop_rate(frequence);

  while (ok()) {

    // a completer ...
    msg.data = "Hello World";

    send_msg(msg);
    pub_msg.publish(msg);
    pub_cmd.publish(msg);

    switch (current_state) {
    case manual:
      break;
    case autonomous:
      move();
      break;
    }

    spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

/**
 *
 * SIGINT handler.
 * Exit code gracefully.
 *
 * @param sig - catched signal.
 *
 */

void sigintHandler(int sig) {
  // Log quit
  ROS_INFO("Exiting program gracefully ...");

  arreter();
  // MESSAGE A PUBLIER le MESSAGE

  // Kill all open subscriptions, publications, service calls, and service
  // servers
  shutdown();
}
