#include <actionlib/server/simple_action_server.h>
#include <projet_robotic/MoveAction.h>
#include <ros/ros.h>

using namespace projet_robotic;
using namespace actionlib;
using namespace ros;

typedef SimpleActionServer<MoveAction> Server;

void execute(const MoveGoalConstPtr &goal, Server *as) {

  MoveResult res;
  MoveFeedback fb;

  ROS_INFO("goal = %f ! ", goal->pos_x);

  int count = 0;
  ros::Rate loop_rate(1); // 1 hz

  bool success = true;
  while (count < goal->pos_x) {

    count++;
    fb.pos_x_intermediaire = count;
    ROS_INFO("POS intermediaire = %f", fb.pos_x_intermediaire);
    as->publishFeedback(fb);

    loop_rate.sleep();
  }

  if (success) {
    res.reached = true;
    ROS_INFO("objectif atteint !");
    as->setSucceeded(res);
  }

  // A complÃ©ter (voir slide serveur d'action 2/2)
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "serveur_action");
  ros::NodeHandle nh;

  Server server(nh, "moving", boost::bind(&execute, _1, &server), false);

  server.start();

  ROS_INFO("serveur demarre et en attente d'un goal ! ");

  spin();

  return 0;
}
