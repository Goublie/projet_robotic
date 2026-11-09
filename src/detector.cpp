#include <algorithm>
#include <cfloat>
#include <cmath>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace ros;
using namespace std;

int NBR_REGION = 8;
int SCAN_SAMPLE = 360;
int ANGLE_REGION = 45;
std::vector<float> min_distances;
Publisher vis_pub;

float DANGEROUS_ZONE = 0.5;
float MEASURABLE_ZONE = 3.5;

float get_minimum(const vector<float>& data, int debut, int fin, int *indice_min);

void select_valid_ranges(vector<float>& ranges, float min_range, float max_range) {
  for (size_t i = 0; i < ranges.size(); i++) {
    if (ranges[i] < min_range || ranges[i] > max_range || isnan(ranges[i]) || isinf(ranges[i])) {
      ranges[i] = 0.0f;
    }
  }
}

void display_data(const vector<float>& distances, const vector<int>& indices) {
  visualization_msgs::MarkerArray marker_array;

  //un point par région
  for (int i = 0; i < NBR_REGION; i++) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "base_scan";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lidar_regions";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    if (distances[i] < 10.0f) {
      double angle_rad = indices[i] * M_PI / 180.0;
      marker.pose.position.x = distances[i] * cos(angle_rad);
      marker.pose.position.y = distances[i] * sin(angle_rad);
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      if (distances[i] < DANGEROUS_ZONE) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      } else if (distances[i] < MEASURABLE_ZONE){
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }
      marker.color.a = 1.0f;
    } 
    else {
      marker.color.a = 0.0f;
    }

    marker_array.markers.push_back(marker);
  }

  vis_pub.publish(marker_array);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& dataLidar) {
  min_distances.clear();
  vector<int> min_indices;

  vector<float> filtered_ranges = dataLidar->ranges;
  select_valid_ranges(filtered_ranges, dataLidar->range_min, dataLidar->range_max);

  for (int i = 0; i < NBR_REGION; i++) {
    int start_index = i * ANGLE_REGION;
    int end_index = (i + 1) * ANGLE_REGION - 1;
    int index_min = -1;

    float min_value = get_minimum(filtered_ranges, start_index, end_index, &index_min);
    min_distances.push_back(min_value);
    min_indices.push_back(index_min);
  }

  display_data(min_distances, min_indices);
}

int main(int argc, char **argv) {
  init(argc, argv, "detector");
  NodeHandle nh;

  Subscriber scan_sub = nh.subscribe("/scan", 100, scanCallback);
  vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/lidar_visualization", 10);

  spin();

  return 0;
}

float get_minimum(const vector<float>& data, int debut, int fin, int *indice_min) {
  float min = FLT_MAX;
  *indice_min = -1;

  for (int i = debut; i <= fin; i++) {
    if (data[i] < min && data[i] > 0.0f) {
      min = data[i];
      *indice_min = i;
    }
  }
  return std::min(10.0f, min);
}