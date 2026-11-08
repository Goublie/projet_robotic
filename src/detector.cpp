#include <algorithm>
#include <ros/ros.h>
#include <string>
#include <vector>

using namespace ros;
using namespace std;

// VARIABLES GLOBALES
int NBR_REGION = 8;
int SCAN_SAMPLE = 360;
int ANGLE_REGION = 45;
std::vector<float> min_distances;

// PROTOTYPE DES FONCTIONS
float get_minimum(vector<float> data, int debut, int fin, int *indiceMin);

// FONCTION CALLBACK
// A COMPLETER//
void scanCallback(/* type  scan */) {

  float min_region_i = FLT_MAX;

  min_distances.clear();

  // A COMPLETER
  // Minimum dans chaque région => détection de l'obstacle / région
}

int main(int argc, char **argv) {
  init(argc, argv, "detector");
  NodeHandle nh;

  Rate loop_rate(/* A COMPLETER */);

  // A COMPLETER
  // Subscriber odom_sub = nh.subscribe("/??", 100, scanCallback);

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