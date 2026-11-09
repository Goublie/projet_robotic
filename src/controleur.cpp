#include <cmath>
#include <csignal>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
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

struct PID {
  double kp;
  double ki;
  double kd;
  double tolerance;
  double erreur_prec;
  double somme_erreur;
  ros::Time temps_prec;
  bool premier_appel;

  PID(double _kp, double _ki, double _kd, double _tolerance) {
    this->kp = _kp;
    this->ki = _ki;
    this->kd = _kd;
    this->tolerance = _tolerance;
    this->erreur_prec = 0.0;
    this->somme_erreur = 0.0;
    this->premier_appel = true;
  }

  void reset() {
    this->somme_erreur = 0.0;
    this->erreur_prec = 0.0;
    this->premier_appel = true;
  }

  double correct(double erreur) {
    if (fabs(erreur) < this->tolerance) {
      return 0.0;
    }

    ros::Time temps_actuel = ros::Time::now();

    if (this->premier_appel) {
      this->temps_prec = temps_actuel;
      this->erreur_prec = erreur;
      this->premier_appel = false;
      return this->kp * erreur;
    }

    double dt = (temps_actuel - this->temps_prec).toSec();
    this->temps_prec = temps_actuel;

    if (dt <= 0.0) {
      dt = 0.1;
    }

    this->somme_erreur += erreur * dt;
    double derivee = (erreur - this->erreur_prec) / dt;
    this->erreur_prec = erreur;

    return this->kp * erreur + this->ki * this->somme_erreur + this->kd * derivee;
  }
};

using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;

void sigintHandler(int sig);
void tourner(double a);

state current_state = autonomous;
Publisher pub_msg;
Publisher pub_vel;
Publisher pub_rst;
String msg;
position current_pos(0, 0, 0);
position start_pos(0, 0, 0);
position target_pos(1, 1, 0);
double linear_speed = LINEAR_SPEED;
double angular_speed = ANGULAR_SPEED;
bool init_travel = true;
bool target_reached = false;
double frequence = FREQUENCY;
Subscriber KeyInput_Sub;
Subscriber Position_Sub;
Subscriber Target_Sub; 

PID pid_distance(0.3, 0.02, 0.05, 0.02);
PID pid_angle(0.3, 0.02, 0.05, 0.05);

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

void reculer(double v) { 
  avancer(-v); 
}

void arreter() { 
  avancer(0.0);
  tourner(0.0); 
}

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
      pid_distance.reset();
      pid_angle.reset();
      ROS_INFO("Mode bascule : AUTONOME");
    } else {
      current_state = manual;
      pid_distance.reset();
      pid_angle.reset();
      arreter();
      ROS_INFO("Mode bascule : MANUEL");
    }
    return;

  case 114: //r reset
    {
      pid_distance.reset();
      pid_angle.reset();
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
    tourner(0.94);
    break;
  case 100: // d
    tourner(-0.94);
    break;

  // COMMANDES DE VITESSE 2
  case 116: // t
    avancer(0.1);
    break;
  case 103: // g
    reculer(0.1);
    break;
  case 102: // f
    tourner(1.88);
    break;
  case 104: // h
    tourner(-1.88);
    break;

  // COMMANDES DE VITESSE 3
  case 105: // i
    avancer(0.15);
    break;
  case 107: // k
    reculer(0.15);
    break;
  case 106: // j
    tourner(2.83);
    break;
  case 108: // l
    tourner(2.83);
    break;

  default:
    arreter();
    break;
  }
}

void targetCallback(geometry_msgs::PoseStamped Target) {
  target_pos.x = Target.pose.position.x;
  target_pos.y = Target.pose.position.y;
  target_pos.angle = tf::getYaw(Target.pose.orientation);
  
  pid_distance.reset();
  pid_angle.reset();
  
  current_state = autonomous;
  ROS_INFO("Nouveau but reçu de RViz : X = %f, Y = %f", target_pos.x, target_pos.y);
  target_reached = false;
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

void collectPosition(nav_msgs::Odometry msg) {
  current_pos.x = msg.pose.pose.position.x;
  current_pos.y = msg.pose.pose.position.y;
  current_pos.angle = tf::getYaw(msg.pose.pose.orientation);

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

bool travel(double distance) {
  if (distance < pid_distance.tolerance) {
    arreter();
    pid_distance.reset();
    return true; // Deplacement termine
  } else {
    double vitesse = pid_distance.correct(distance);
    avancer(vitesse);
    return false; // Deplacement en cours
  }
}

bool turnTo(double targeted_angle) {
  double erreur_angle = cal_delta_Theta(targeted_angle, current_pos.angle);

  if (fabs(erreur_angle) < pid_angle.tolerance) {
    arreter();
    pid_angle.reset();
    return true;
  } else {
    double vitesse_angulaire = pid_angle.correct(erreur_angle);
    tourner(vitesse_angulaire);
    return false;
  }
}

void goTo(position target_pos)
{
  //On se tourne vers la position cible
  //Puis on avance de la distance qui les sépare
  double targeted_angle = atan2( (target_pos.y-current_pos.y) , (target_pos.x-current_pos.x) );
  position direction = getVector(target_pos);
  double distance = direction.getSize();
  
  if (!target_reached){
    if(turnTo(targeted_angle)){
      if(travel(distance)){
        target_reached = true; 
      }
    }
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
  Target_Sub = nh.subscribe("/move_base_simple/goal", 10, targetCallback);

  nh.getParam("freq", frequence);
  int cap_log = std::round(2 * frequence) - 1;

  nh.getParam("Kp", pid_distance.kp);
  nh.getParam("Kd", pid_distance.kd);
  nh.getParam("Ki", pid_distance.ki);

  nh.getParam("Kp_angle", pid_angle.kp);
  nh.getParam("Kd_angle", pid_angle.kd);
  nh.getParam("Ki_angle", pid_angle.ki);

  nh.getParam("x", target_pos.x);
  nh.getParam("y", target_pos.y);

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
      goTo(target_pos);
      //turnTo(target_pos.angle);
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
