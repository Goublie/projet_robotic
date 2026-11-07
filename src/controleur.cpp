#include <cmath>
#include <csignal>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/tf.h>

#define FREQUENCY 7.5
#define MAX_LINEAR_SPEED 0.15
#define MAX_ANGULAR_SPEED 2.84
#define LINEAR_SPEED 0.1
#define ANGULAR_SPEED 0.1
#define AV 1
#define ARR 2

int cpt_log = 0;
int cpt = 0;

enum state { manual, autonomous };

struct position {
  double x;
  double y;
  double angle;

  position(double _x, double _y, double _angle) {
    this->x = _x;
    this->y = _y;
    this->angle = _angle;
  }
  double getSize() { return sqrt(x * x + y * y); }
};

using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;

void sigintHandler(int sig);
double pid(double erreur, double kp, double ki, double kd, double tolerance, ros::Time &temps_prec, double &erreur_prec, double &somme_erreur, bool &premier_appel);
void reset_pid();

state current_state = autonomous;
Publisher pub_msg;
Publisher pub_vel;
Publisher pub_rst;
String msg;
position current_pos(0, 0, 0);
position target_pos(0, 0, 2.7);
double linear_speed = LINEAR_SPEED;
double angular_speed = ANGULAR_SPEED;
double frequence = FREQUENCY;
Subscriber KeyInput_Sub;
Subscriber Position_Sub;

double Kp = 0.3;
double Ki = 0.02;
double Kd = 0.05;
double erreur_prec = 0.0;
double somme_erreur = 0.0;
ros::Time temps_prec;
bool premier_appel = true;
double cap_error = 0.02;

double Kp_angle = 0.3;
double Ki_angle = 0.02;
double Kd_angle = 0.05;
double erreur_prec_angle = 0.0;
double somme_erreur_angle = 0.0;
ros::Time temps_prec_angle;
bool premier_appel_angle = true;
double cap_angle = 0.05;

// L'impl des fonctions
void avancer(double v) {
  Twist cmd_vel1;
  if (fabs(v) > MAX_LINEAR_SPEED) {
    if (v < 0) {
      v = -MAX_LINEAR_SPEED;
    }
    else {
    v = MAX_LINEAR_SPEED;
    }
  }
  cmd_vel1.linear.x = v;
  cmd_vel1.angular.z = 0;

  pub_vel.publish(cmd_vel1);
}

void reculer(double v) { avancer(-v); }

void arreter() { avancer(0.0); }

void tourner(double a) {
  Twist cmd_vel2;
    if (fabs(a) > MAX_ANGULAR_SPEED) {
      if (a<0) {
        a = -MAX_ANGULAR_SPEED;
      } 
      else {
        a = MAX_ANGULAR_SPEED;
      }
  }
  cmd_vel2.linear.x = 0;
  cmd_vel2.angular.z = a;

  pub_vel.publish(cmd_vel2);
}

void send_msg(String msg) {
  ROS_INFO("Publisher : %s", msg.data.c_str());
  pub_msg.publish(msg);
}

void kbCallback(std_msgs::Int8 kbInput) {
  int key = kbInput.data;

  switch (key) {
  case 109: //m change de mode
    if (current_state == manual) {
      current_state = autonomous;
      reset_pid();
      ROS_INFO("Mode bascule : AUTONOME");
    } else {
      current_state = manual;
      reset_pid();
      arreter();
      ROS_INFO("Mode bascule : MANUEL");
    }
    return;

  case 114: //r reset
    {
      reset_pid();
      std_msgs::Empty reset_msg;
      pub_rst.publish(reset_msg);
      ROS_INFO("Reset envoye");
    }
    return;
  }

  // Si on n'est pas en mode manuel, on ignore les touches de commande de déplacement
  if (current_state != manual) {
    return;
  }

  // COMMANDES DE DEPLACEMENT (Actives uniquement en mode manuel)
  switch (key) {
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

  default:
    arreter();
    break;
  }
}
  
void move() {
  ROS_INFO("move");

  switch (cpt) {
  case 0:
    avancer(MAX_LINEAR_SPEED);
    break;
  case 1:
    arreter();
    break;
  case 2:
    reculer(MAX_LINEAR_SPEED);
    break;
  case 3:
    arreter();
    break;
  }

  cpt++;
  cpt %= 4;
}

void collectPosition(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pos.x = msg->pose.pose.position.x;
  current_pos.y = msg->pose.pose.position.y;
  current_pos.angle = tf::getYaw(msg->pose.pose.orientation);

  ROS_INFO("Position actuelle : X = %f, Y = %f, theta = %f", current_pos.x,
           current_pos.y, current_pos.angle);
}

position getVector(position target_pos) {
  position vector(0, 0, 0);
  vector.x = target_pos.x - current_pos.x;
  vector.y = target_pos.y - current_pos.y;
  vector.angle = target_pos.angle - current_pos.angle;
  return vector;
}

double pid(double erreur, double kp, double ki, double kd, double tolerance, ros::Time &temps_prec, double &erreur_prec, double &somme_erreur, bool &premier_appel) {
  if (fabs(erreur) < tolerance) {
    return 0.0;
  }

  ros::Time temps_actuel = ros::Time::now();

  if (premier_appel) {
    temps_prec = temps_actuel;
    erreur_prec = erreur;
    premier_appel = false;
    return kp * erreur;
  }

  double dt = (temps_actuel - temps_prec).toSec();
  temps_prec = temps_actuel;

  if (dt <= 0.0) {
    dt = 0.1;
  }

  somme_erreur += erreur * dt;
  double derivee = (erreur - erreur_prec) / dt;
  erreur_prec = erreur;

  return kp * erreur + ki * somme_erreur + kd * derivee;
}

void reset_pid() {
  somme_erreur = 0.0;
  erreur_prec = 0.0;
  premier_appel = true;
}

void reset_pid_angle() {
  somme_erreur_angle = 0.0;
  erreur_prec_angle = 0.0;
  premier_appel_angle = true;
}

float cal_delta_Theta(float _theta, float _theta0) {

  float theta, theta0, delta_theta;

  if  (_theta < 0)  theta = _theta + 2 * M_PI;
  else theta = _theta;

  if  (_theta0 < 0)  theta0 = _theta0 + 2 * M_PI;
  else theta0 = _theta0;

  delta_theta = theta - theta0;

  if (delta_theta < -M_PI) {
      delta_theta = delta_theta + 2 * M_PI;
  } else {
      if (delta_theta >= M_PI) {
        delta_theta = delta_theta - 2 * M_PI;
      }
    }
    return delta_theta;
}

void goTo(position target_pos) {
  position vector = getVector(target_pos);
  double erreur = target_pos.x - current_pos.x;

  if (fabs(erreur) < 0.02) {
    arreter();
    reset_pid();
  } else {
    double vitesse = pid(erreur, Kp, Ki, Kd, 0.02, temps_prec, erreur_prec, somme_erreur, premier_appel);

    vitesse = MAX_LINEAR_SPEED;

    avancer(vitesse);
  }
}

void turnTo(position target_pos){
  double erreur_angle = cal_delta_Theta(target_pos.angle, current_pos.angle);

  if (fabs(erreur_angle) < cap_angle) {
    arreter();
    reset_pid_angle();
  } else {
    double vitesse_angulaire = pid(erreur_angle, Kp_angle, Ki_angle, Kd_angle, cap_angle, temps_prec_angle, erreur_prec_angle, somme_erreur_angle, premier_appel_angle);

    tourner(vitesse_angulaire);
  }
}

int main(int argc, char **argv) {

  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "controleur", ros::init_options::NoSigintHandler);

  NodeHandle nh;
  pub_msg = nh.advertise<String>("/log", 1000);
  pub_vel = nh.advertise<Twist>("/cmd_vel", 10);
  pub_rst = nh.advertise<std_msgs::Empty>("/reset", 10);
  KeyInput_Sub = nh.subscribe("/keyboard_input", 1000, kbCallback);
  Position_Sub = nh.subscribe("/odom", 1000, collectPosition);

  nh.getParam("freq", frequence);
  int cap_log = std::round(2 * frequence) -1;

  nh.getParam("Kp", Kp);
  nh.getParam("Kd", Kd);
  nh.getParam("Ki", Ki);

  nh.getParam("Kp_angle", Kp_angle);
  nh.getParam("Kd_angle", Kd_angle);
  nh.getParam("Ki_angle", Ki_angle);

  std::string state_str;
  if (nh.getParam("actual_state", state_str) || nh.getParam("current_state", state_str)) {
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

    if (cpt_log >= cap_log){
      send_msg(msg);
      cpt_log = 0;
    }
    else {
      cpt_log ++;
    }

    switch (current_state) {
    case manual:
      break;
    case autonomous:
      // move();
      //goTo(target_pos);
      turnTo(target_pos);
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
