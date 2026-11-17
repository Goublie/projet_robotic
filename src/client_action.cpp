#include "ros/init.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose2D.h>
#include <projet_robotic/MoveAction.h>
#include <nav_msgs/Path.h>

using namespace actionlib;
using namespace ros;
using namespace std;
using namespace geometry_msgs;
using namespace projet_robotic;

typedef SimpleActionClient<MoveAction> Client;

Pose2D position;
MoveGoal goal;
std::vector<Pose2D> trajectoire;

Subscriber path_sub;

void spinThread() { 
  spin(); 
}

void doneCb(const SimpleClientGoalState &s, const MoveResultConstPtr &result) {
  if (s == SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Action terminee avec succes ! Result = %d, distance = %f m, temps = %f s", result->reached, result->distance_totale, result->temps_ecoule);

    if (trajectoire.empty()) {
      ROS_INFO("Trajectoire complétée.");
    }
  } else {
    ROS_INFO("Action interrompue.");
  }
}

void activeCb() { 
  ROS_INFO("Goal actif"); 
}

void feedback_callback(const MoveFeedbackConstPtr &feedback) {
  ROS_INFO("Feedback recu : X = %f, Y = %f, distance restante = %f", 
           feedback->pos_x_intermediaire, feedback->pos_y_intermediaire, feedback->distance_restante);
}

void pathCallback(nav_msgs::Path path){
  trajectoire.clear();
  for (int i=0; i<path.poses.size(); i++){
    position.x = path.poses[i].pose.position.x;
    position.y = path.poses[i].pose.position.y;
    trajectoire.push_back(position);
  }

}

int main(int argc, char **argv) {
  init(argc, argv, "planner");
  NodeHandle nh;
  
  Client client("moving", true);
  
  path_sub = nh.subscribe("/consigne_trajectoire", 10, pathCallback);
  client.waitForServer();
  Rate loop_rate(10);
  while (ok()) {
  
    // Si on a des points dans notre file d'attente
    if (!trajectoire.empty()) {
      goal.pos_x = trajectoire[0].x;
      goal.pos_y = trajectoire[0].y;
      trajectoire.erase(trajectoire.begin());
      ROS_INFO("Envoi du point : X = %f, Y = %f (restants : %zu)", goal.pos_x, goal.pos_y, trajectoire.size());
      client.sendGoal(goal);
      
      // On attend que le robot arrive avant de passer au point suivant
      client.waitForResult(Duration(120.0));
      if (client.getState() == SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Point atteint !");
        if (trajectoire.empty()) {
          ROS_INFO("Trajectoire entièrement terminée !");
        }
      }
    }
    spinOnce();
    loop_rate.sleep();
  }
  return 0;
}