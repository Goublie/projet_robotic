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
  ros::spin(); 
}

void doneCb(const SimpleClientGoalState &s, const MoveResultConstPtr &result) {
  if (s == SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Action terminee avec succes ! Result = %d, distance = %f m, temps = %f s", result->reached, result->distance_totale, result->temps_ecoule);
    
    //client.sendGoal(goal, &doneCb, &activeCb, &feedback_callback);
    //client.waitForResult(ros::Duration(120.0));

    //if (trajectoire.empty()){
    //  ROS_INFO("Trajectoire achevée.");
    //}
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
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  nh.subscribe("/consigne_trajectoire",100, pathCallback);

  boost::thread spin_thread(&spinThread);

  Client client("moving", true);
  client.waitForServer();

  if (argc > 2) {
    goal.pos_x = atof(argv[1]);
    goal.pos_y = atof(argv[2]);
  } else if (argc > 1) {
    goal.pos_x = atof(argv[1]);
    goal.pos_y = 0.0;
  } else {
    goal.pos_x = 1.0;
    goal.pos_y = 0.5;
  }

  ROS_INFO("Envoi du goal : X = %f, Y = %f", goal.pos_x, goal.pos_y);
  client.sendGoal(goal, &doneCb, &activeCb, &feedback_callback);
  client.waitForResult(ros::Duration(120.0));

  if (client.getState() == SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("L'objectif est atteint !");
  } else {
    client.cancelGoal();
  }

  ROS_INFO("Current State: %s", client.getState().toString().c_str());

  ros::shutdown();
  spin_thread.join();
  return 0;
}
