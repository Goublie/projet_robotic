#include <csignal>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h> //ajout
#include <string>
#include <std_msgs/Int8.h>
// a completer

double var_freq = nh.getParam("freq", var_freq);

#define FREQUENCY var_freq
#define MAX_LINEAR_SPEED 0.15
#define MAX_ANGULAR_SPEED 2.84
#define LINEAR_SPEED 0.1
#define ANGULAR_SPEED 0.1
#define AV 1
#define ARR 2
int cpt = 0;

using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;

// Les prototypes des fonctions
void sigintHandler(int sig);

// Les variables globales
Publisher pub_msg;
Publisher cmd;
String msg;
double linear_speed = LINEAR_SPEED;
double angular_speed = ANGULAR_SPEED;
double frequence = FREQUENCY;
Subscriber KeyInput_Sub;

// L'impl des fonctions
void arreter() {
  avancer(0.0);
}
void avancer(double v) {
  Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = 0;

  geometry_msgs.publish(cmd);
}

void tourner(double a) {
  Twist cmd;
  cmd.linear.x = 0;
  cmd.angular.z = a;

  geometry_msgs.publish(cmd);
}

void send_msg(String msg) {

  ROS_INFO("Publisher : %s", msg.data.c_str());
  pub_msg.publish(msg);
  // a completer...
}

void kbCallback(std_msgs::Int8 kbInput) {
  match (kbInput.data) {
    case kbInput.data == 122 : // z
      avancer(0.2);
      break;
    case kbInput.data == 115 : // s
      reculer(0.2);
      break;
    case kbInput.data == 113 : // q
      trouner(0.2);
      break; 
    case kbInput.data == 100 : // d
      trouner(-0.2);
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

int main(int argc, char **argv) {

  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "controleur", ros::init_options::NoSigintHandler);

  NodeHandle nh;
  pub_msg = nh.advertise<String>("/log", 1000);
  cmd = nh.advertise<Twist>("/cmd_vel", 10);

  
  // a completer...
  
  Rate loop_rate(frequence);
  
  while (ok()) {

    // a completer ...
    msg.data = "Hello World";
    
    send_msg(msg);
    pub_msg.publish(msg);
    cmd.publish(msg);
    
    //INTERCEPTION CLAVIER

    KeyInput_Sub = nh.subscribe("/keyboard_input", 1000, kbCallback);

    //INTERCEPTION CLAVIER

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
