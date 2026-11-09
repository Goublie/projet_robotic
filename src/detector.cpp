#include <algorithm>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

using namespace ros;
using namespace std;

// VARIABLES GLOBALES
int NBR_REGION = 8;
int SCAN_SAMPLE = 360;
int ANGLE_REGION = 45;
std::vector<float> min_distances;
visualization_msgs::Marker marker;
Publisher vis_pub;

// PROTOTYPE DES FONCTIONS
float get_minimum(vector<float> data, int debut, int fin, int *indiceMin);

void select_valid_ranges(vector<float>& ranges, float min_range, float max_range){
  for (int i = 0; i<ranges.size(); i++){
    if (ranges[i]<min_range || ranges[i]>max_range){
      ranges[i] = 0.0;
    }
  } 
}

void display_data(){
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 1;
  marker.pose.position.y = 1;
  marker.pose.position.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  vis_pub.publish( marker );
}

// FONCTION CALLBACK
// A COMPLETER//
void scanCallback(sensor_msgs::LaserScan dataLidar) {

  float min_region_i = FLT_MAX;

  min_distances.clear();

  // A COMPLETER
  // Minimum dans chaque région => détection de l'obstacle / région
  
  //extraction des distances par région
  vector<float> filtered_ranges = dataLidar.ranges;
  for (int i = 0; i<NBR_REGION; i++){
    int start_index = i*ANGLE_REGION;
    int end_index = (i+1) * ANGLE_REGION -1;
    int index_min = -1;

    float min_value = get_minimum(filtered_ranges, start_index, end_index, &index_min);
    min_distances.push_back(min_value);
  }

  display_data();
}

int main(int argc, char **argv) {
  init(argc, argv, "detector");
  NodeHandle nh;

  Rate loop_rate(5);

  // A COMPLETER
  Subscriber odom_sub = nh.subscribe("/scan", 100, scanCallback);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  spin();

  return 0;
}

float get_minimum(vector<float> data, int debut, int fin, int *indice_min) {

  float min = FLT_MAX;

  for (int i = debut; i <= fin; i++) {
    if (data[i]<min && data[i]> 0.0) {
      min = data[i];
      *indice_min = i;
    }
  }
  return std::min(10.0f, min);
}