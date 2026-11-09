#include "ros/init.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose2D.h>
#include <tb3_projet_vos_initiales/MoveAction.h>

using namespace actionlib;
using namespace ros;
using namespace std;
using namespace geometry_msgs;
using namespace tb3_projet_vos_initiales;

typedef SimpleActionClient<MoveAction> Client;

Pose2D position;
MoveGoal goal;

/**
###  Vecteur pour y stocker des positions
###  OpÃ©rations possibles :
    trajectoire.size() : retourne la taille du vecteur
    trajectoire.push_back(position) : insÃ¨re une positon Ã  la fin du vecteur
    trajectoire.erase(trajectoire.begin()) : supprime le premier Ã©lement du
                                             vecteur
    trajectoire[0] : permet l'accÃ¨s au premier Ã©lement du vecteur (position
                     de type Pose2D)
**/
std::vector<Pose2D> trajectoire;

void spinThread() { ros::spin(); }

/*
This method is called when the goal completes
*/
void doneCb(const SimpleClientGoalState &s, const MoveResultConstPtr &result) {
  if (s == SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("action se termine avec success ! result = %d", result->reached);
  } else
    ROS_INFO("Action is preempted");
}

/*
 This method is called when the goal becomes active
*/
void activeCb() { ROS_INFO("Goal just went active"); }

/*
This method is called whenever there is data present on feedback channel
*/
void feedback_callback(const MoveFeedbackConstPtr &feedback) {
  ROS_INFO("Feedback recu : %f !", feedback->pos_x_intermediaire);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  boost::thread spin_thread(&spinThread);

  // A ComplÃ©ter...

  Client client("moving", true); // true -> don't need ros::spin()
  client.waitForServer();

  goal.pos_x = 10;

  // A complÃ©ter...
  client.sendGoal(goal, &doneCb, &activeCb, &feedback_callback);
  client.waitForResult(ros::Duration(15.0));

  if (client.getState() == SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("L'objectif est maintenant atteint !");
  else
    client.cancelGoal();

  ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
  // ... voir slide "les actions"

  spin_thread.join();
  return 0;
}
