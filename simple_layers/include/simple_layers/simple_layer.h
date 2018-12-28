#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

namespace simple_layer_namespace
{

class SimpleLayer : public costmap_2d::Layer
{
public:
  SimpleLayer();
  ~SimpleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  double dan_rx[10],dan_ry[10],dan_lx[10],dan_ly[10];
  double mark_rx[10][50][500];
  double mark_ry[10][50][500];
  double mark_lx[10][50][500];
  double mark_ly[10][50][500];
  size_t ro;
  size_t lo;
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  ros::Subscriber sub;
};
}
#endif