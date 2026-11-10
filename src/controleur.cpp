#include <cmath>
#include <csignal>
#include <string>
#include <tf/tf.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h>
#include <projet_robotic/MoveAction.h>
#include <projet_robotic/set_behavior.h>

using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;

typedef actionlib::SimpleActionServer<projet_robotic::MoveAction> ActionServer;

//==================
//=== CONSTANTES ===
//==================
#define FREQUENCY 7.5
#define MAX_LINEAR_SPEED 0.15
#define MAX_ANGULAR_SPEED 2.84
#define LINEAR_SPEED 0.1
#define ANGULAR_SPEED 0.1
#define AV 1
#define ARR 2

//==================
//=== STRUCTURES ===
//==================
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

//==========================
//=== VARIABLES GLOBALES ===
//==========================

// NAVIGATION
state current_state = autonomous;
position current_pos(0, 0, 0);
position start_pos(0, 0, 0);
position target_pos(1, 1, 0);
bool init_travel = true;
bool target_reached = false;
bool obstacle_front = false;
double frequence = FREQUENCY;
int cpt_log = 0;
int cpt = 0;
String msg;

// PID
double cap_linear_speed = MAX_LINEAR_SPEED / 2.0;
double cap_angular_speed = MAX_ANGULAR_SPEED / 2.0;
PID pid_distance(0.3, 0.02, 0.05, 0.02);
PID pid_angle(0.3, 0.02, 0.05, 0.05);

// PUBLISHERS
Publisher pub_msg;
Publisher pub_vel;
Publisher pub_rst;

// SUBSCRIBERS
Subscriber KeyInput_Sub;
Subscriber Position_Sub;
Subscriber Target_Sub;
Subscriber Lidar_Sub;

//===============================
//=== PROTOTYPES DE FONCTIONS ===
//===============================
void sigintHandler(int sig);
void avancer(double v);
void reculer(double v);
void arreter();
void tourner(double a);
void send_msg(String msg);
float cal_delta_Theta(float _theta, float _theta0);
position getVector(position target_pos);
bool travel(double distance);
bool turnTo(double targeted_angle);
void goTo(position target_pos);
void move();
void kbCallback(std_msgs::Int8 kbInput);
void collectPosition(nav_msgs::Odometry msg);
void targetCallback(geometry_msgs::PoseStamped Target);
void lidarCallback(const visualization_msgs::MarkerArray &markers);
bool setBehaviorCallback(projet_robotic::set_behavior::Request &req, projet_robotic::set_behavior::Response &res);
void executeAction(const projet_robotic::MoveGoalConstPtr &goal, ActionServer *as);

void avancer(double v) {
  if (obstacle_front && v > 0) {
    ROS_INFO("Objet détecté.");
    arreter();
    return;
  }

  Twist cmd_vel1;
  if (fabs(v) > cap_linear_speed) {
    if (v < 0) {
      v = -cap_linear_speed;
    } else {
      v = cap_linear_speed;
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
  if (fabs(a) > cap_angular_speed) {
    if (a < 0) {
      a = -cap_angular_speed;
    } else {
      a = cap_angular_speed;
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

float cal_delta_Theta(float _theta, float _theta0) {
  float theta, theta0, delta_theta;

  if (_theta < 0) theta = _theta + 2 * M_PI;
  else theta = _theta;

  if (_theta0 < 0) theta0 = _theta0 + 2 * M_PI;
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

position getVector(position target_pos) {
  position vector(0, 0, 0);
  vector.x = target_pos.x - current_pos.x;
  vector.y = target_pos.y - current_pos.y;
  vector.angle = target_pos.angle - current_pos.angle;
  return vector;
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

void goTo(position target_pos) {
  //On se tourne vers la position cible
  //Puis on avance de la distance qui les sépare
  double targeted_angle = atan2((target_pos.y - current_pos.y), (target_pos.x - current_pos.x));
  position direction = getVector(target_pos);
  double distance = direction.getSize();

  if (!target_reached) {
    if (turnTo(targeted_angle)) {
      if (travel(distance)) {
        target_reached = true; 
      }
    }
  }
}

void move() {
  ROS_INFO("move");

  switch (cpt) {
  case 0:
    avancer(cap_linear_speed);
    break;
  case 1:
    arreter();
    break;
  case 2:
    reculer(cap_linear_speed);
    break;
  case 3:
    arreter();
    break;
  }

  cpt++;
  cpt %= 4;
}

//=================
//=== CALLBACKS ===
//=================
void kbCallback(std_msgs::Int8 kbInput) {
  int key = kbInput.data;

  switch (key) {
  case 109: // m change de mode
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

  case 114: // r reset
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

void collectPosition(nav_msgs::Odometry msg) {
  current_pos.x = msg.pose.pose.position.x;
  current_pos.y = msg.pose.pose.position.y;
  current_pos.angle = tf::getYaw(msg.pose.pose.orientation);

  ROS_INFO("Position actuelle : X = %f, Y = %f, theta = %f", current_pos.x,
           current_pos.y, current_pos.angle);
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

void lidarCallback(const visualization_msgs::MarkerArray &markers) {
  bool obstacle = false;
  for (size_t i = 0; i < markers.markers.size(); ++i) {
    int id = markers.markers[i].id;
    // Les 4 zones de devant sont 0, 1, 6 et 7
    if (id == 0 || id == 1 || id == 6 || id == 7) {
      // Zone rouge : marker rouge actif (r = 1.0, g = 0.0, a > 0.0)
      if (markers.markers[i].color.r > 0.9f) {
        obstacle = true;
        break;
      }
    }
  }

  obstacle_front = obstacle;
}

//===============
//=== SERVICE ===
//===============
bool setBehaviorCallback(projet_robotic::set_behavior::Request &req, projet_robotic::set_behavior::Response &res) {
  switch (req.mode) {
    case 0:
      current_state = manual;
      arreter();

      pid_distance.reset();
      pid_angle.reset();
      
      res.success = true;
      res.mode_courant = 0;
      
      ROS_INFO("Mode Manu");
      break;

    case 1:
      current_state = autonomous;
      
      cap_linear_speed = MAX_LINEAR_SPEED / 2.0;
      cap_angular_speed = MAX_ANGULAR_SPEED / 2.0;
      
      res.success = true;
      res.mode_courant = 1;
      
      ROS_INFO("Mode Auto");
      break;

    case 2:
      current_state = autonomous;
      
      cap_linear_speed = 0.15;
      cap_angular_speed = MAX_ANGULAR_SPEED;
      
      res.success = true;
      res.mode_courant = 2;
      
      ROS_INFO("Mode Turbo");
      break;

    default:
      res.success = false;
      if (current_state == manual) {
        res.mode_courant = 0;
      } else {
        res.mode_courant = (cap_linear_speed == 0.15) ? 2 : 1;
      }
      break;
  }
  return true;
}

//==============
//=== ACTION ===
//==============
void executeAction(const projet_robotic::MoveGoalConstPtr &goal, ActionServer *as) {
  target_pos.x = goal->pos_x;
  target_pos.y = goal->pos_y;
  target_reached = false;
  current_state = autonomous;
  pid_distance.reset();
  pid_angle.reset();

  position start = current_pos;
  ros::Time start_time = ros::Time::now();

  ros::Rate action_rate(2);
  while (ros::ok() && !target_reached) {
    if (as->isPreemptRequested() || current_state != autonomous) {
      arreter();
      as->setPreempted();
      return;
    }

    projet_robotic::MoveFeedback fb;
    fb.pos_x_intermediaire = current_pos.x;
    fb.pos_y_intermediaire = current_pos.y;
    fb.distance_restante = getVector(target_pos).getSize();
    as->publishFeedback(fb);

    action_rate.sleep();
  }

  if (target_reached) {
    projet_robotic::MoveResult res;
    res.reached = true;
    res.distance_totale = sqrt(pow(current_pos.x - start.x, 2) + pow(current_pos.y - start.y, 2));
    res.temps_ecoule = (ros::Time::now() - start_time).toSec();
    as->setSucceeded(res);
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "controleur", ros::init_options::NoSigintHandler);

  NodeHandle nh;

  // PUBLISHERS 
  pub_msg = nh.advertise<String>("/log", 1000);
  pub_vel = nh.advertise<Twist>("/cmd_vel", 10);
  pub_rst = nh.advertise<std_msgs::Empty>("/reset", 10);

  // SUBSCRIBERS
  KeyInput_Sub = nh.subscribe("/keyboard_input", 1000, kbCallback);
  Position_Sub = nh.subscribe("/odom", 1000, collectPosition);
  Target_Sub = nh.subscribe("/move_base_simple/goal", 10, targetCallback);
  Lidar_Sub = nh.subscribe("/lidar_visualization", 10, lidarCallback);

  // SERVICES & ACTIONS
  ServiceServer service_behavior = nh.advertiseService("/set_behavior", setBehaviorCallback);
  ActionServer action_server(nh, "moving", boost::bind(&executeAction, _1, &action_server), false);
  action_server.start();

  // PARAMETRES
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

  Rate loop_rate(frequence);

  while (ok()) {
    msg.data = "Hello World";

    if (cpt_log >= cap_log) {
      send_msg(msg);
      cpt_log = 0;
    } else {
      cpt_log++;
    }

    switch (current_state) {
    case manual:
      break;
    case autonomous:
      goTo(target_pos);
      break;
    }

    spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

//======================
//=== GESTION SIGINT ===
//======================
void sigintHandler(int sig) {
  ROS_INFO("Exiting program gracefully ...");
  arreter();
  shutdown();
}