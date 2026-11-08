#include <algorithm>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>

using namespace ros;
using namespace std;

// VARIABLES GLOBALES
int NBR_REGION = 8;
int SCAN_SAMPLE = 360;
int ANGLE_REGION = 45;
std::vector<float> min_distances;

// PROTOTYPE DES FONCTIONS
float get_minimum(vector<float> data, int debut, int fin, int *indiceMin);

void select_valid_ranges(vector<float>& ranges, float min_range, float max_range){
  for (int i = 0; i<ranges.size(); i++){
    if (ranges[i]<min_range || ranges[i]>max_range){
      ranges[i] = 0.0;
    }
  } 
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
}

int main(int argc, char **argv) {
  init(argc, argv, "detector");
  NodeHandle nh;

  Rate loop_rate(5);

  // A COMPLETER
  Subscriber odom_sub = nh.subscribe("/scan", 100, scanCallback);

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