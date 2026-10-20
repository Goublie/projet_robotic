#include <csignal>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h> //ajout
#include <string>
#include <std_msgs/Int8.h>
// a completer

#define FREQUENCY 0.5
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
Publisher pub_vel;
Publisher cmd;
String msg;
double linear_speed = LINEAR_SPEED;
double angular_speed = ANGULAR_SPEED;
double frequence = FREQUENCY;
Subscriber KeyInput_Sub;

// L'impl des fonctions
void avancer(double v) {
  Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = 0;

  pub_vel.publish(cmd);
}

void reculer(double v) {
  avancer(-v);
}

void arreter() {
  avancer(0.0);
}

void tourner(double a) {
  Twist cmd;
  cmd.linear.x = 0;
  cmd.angular.z = a;

  pub_vel.publish(cmd);
}

void send_msg(String msg) {

  ROS_INFO("Publisher : %s", msg.data.c_str());
  pub_msg.publish(msg);
  // a completer...
}

void kbCallback(std_msgs::Int8 kbInput) {
  switch (kbInput.data) {
   //COMMANDES DE VITESSE 1
      case 122 : // z
      avancer(0.2);
      break;
    case 115 : // s
      reculer(0.2);
      break;
    case 113 : // q
      tourner(0.2);
      break;
    case 100 : // d
      tourner(-0.2);
      break;
  //COMMANDES DE VITESSE 2

     case 116 : // t
      avancer(0.4);
      break;
    case 103 : // g
      reculer(0.4);
      break;
    case 102 : // f
      tourner(0.4);
      break;
    case 104 : // h
      tourner(-0.4);
      break;

    //COMMANDES DE VITESSE 3

    case 105 : // i
      avancer(0.6);
      break;
    case 107 : // k
      reculer(0.6);
      break;
    case 106 : // j
      tourner(0.6);
      break;
    case 108 : // l
      tourner(-0.6);
      break;
      //COMMANDES DE VITESSE 4

     case 56 : // 8
      avancer(0.2);
      break;
    case 53 : // 5
      reculer(0.2);
      break;
    case 52 : // 4
      tourner(0.2);
      break;
    case 54 : // 6
      tourner(-0.2);
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
