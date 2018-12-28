#define _USE_MATH_DEFINES
#include <math.h>
#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)


using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}
SimpleLayer::~SimpleLayer()
{
  sub.shutdown();
}

void SimpleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  size_t i=0;
  double angle_min = msg->angle_min;
  double angle_max = msg->angle_max;
  double angle_increment = msg->angle_increment;

  double range_min = msg->range_min;
  double range_max = msg->range_max;
  double r[1000];
  double x[1000];
  double y[1000];
  double theta[1000];

  size_t n=0;
  double new_r[1000];
  double new_x[1000];
  double new_y[1000];
  double new_theta[1000];
  size_t end_p=0;


  size_t center_p=0;
  size_t occ_lp[100];
  size_t occ_rp[100];

/*ファイルを開く
  FILE *fp;
  fp =fopen("URG_Data.dat","w");*/

 //std::cout << "range_max: " << range_max << std::endl;　

  size_t data_length = (angle_max - angle_min) / angle_increment;
  for ( i = 0; i < data_length; i++) {

     //1点の位置を計算
    r[i] = std::min(range_max, std::max(range_min, (double)msg->ranges[i]));
    x[i] = r[i] * std::cos(angle_min + i * angle_increment);
    y[i] = r[i] * std::sin(angle_min + i * angle_increment);
    theta[i] = angle_min + i * angle_increment;

//座標を表示
  //std::cout <<"x"<<i <<":"<< x[i] << std::endl;
  //std::cout <<"y"<<i <<":"<< y[i] << std::endl;


//20cm以下の点を排除して、再定義
   if(r[i]>=0.2){
    new_r[n]=r[i];
    new_x[n]=x[i]; 
    new_y[n]=y[i];
    new_theta[n]=theta[i];
//fprintf(fp,"%zu %lf %lf %lf %lf \n",n,new_r[n],new_x[n],new_y[n],new_theta[n]);
    n=n+1;
     }
  }//全点の位置計測と再定義終わり

//中心の検出
for(i=1; i<n; i++){
center_p = i;
if(theta[i]>0){break;}
}

//fclose(fp);//ファイル閉じる

//ファイルを開く
//fp =fopen("URG_Occ.dat","w");

//右オクルージョンの判定
ro=0;
for(i=30; i<center_p-1; i++){
if((new_r[i+1]-new_r[i])>1||(new_r[i]-new_r[i+1])>1){
 occ_rp[ro]=i;
//fprintf(fp,"%zu \n",occ_rp[ro]);
 ro=ro+1;
}}

//左オクルージョンの判定
lo=0;
for(i=center_p; i<n-30; i++){
if((new_r[i+1]-new_r[i])>1||(new_r[i]-new_r[i+1])>1){
 occ_lp[lo]=i+1;
//fprintf(fp,"%zu \n",occ_lp[lo]);
 lo=lo+1;
}}

//右危険領域の中心
//double dan_rx[100];
//double dan_ry[100];
for(i=0; i<ro; i++){
  dan_rx[i]=new_x[occ_rp[i]];
  dan_ry[i]=new_y[occ_rp[i]]+0.4* std::tan(new_theta[occ_rp[i]]);
  std::cout <<"right danger x:"<<dan_rx[i] <<" y"<< dan_ry[i] << std::endl;
  //fprintf(fp,"%lf %lf \n",dan_rx[i],dan_ry[i]);
}

//左危険領域の中心
//double dan_lx[100];
//double dan_ly[100];
for(i=0; i<lo; i++){
  dan_lx[i]=new_x[occ_lp[i]];
  dan_ly[i]=new_y[occ_lp[i]]+0.4* std::tan(new_theta[occ_lp[i]]);
 std::cout <<"left danger x:"<<dan_lx[i] <<" y"<< dan_ly[i] << std::endl;
  //fprintf(fp,"%lf %lf \n",dan_lx[i],dan_ly[i]);
}
  //simple_layer.hの中でメンバ変数として定義したdan_rx配列の中に適当な値を突っ込む（テスト用）
  //ここでメンバ変数に入れた値をupdateBoundsやupdateCostsで使う
 
}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  sub = g_nh.subscribe("/scan", 1, &SimpleLayer::laserScanCallback, this);
}


void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

int o=0;//オクルージョンの数
int i=0;//何番目の円か
int n=0;//一周のうち、何番目の点か

//右危険領域のコスト変更範囲の決定
for(o=0; o<ro; o++){//オクルージョンの数だけ試行する
 //中心
 mark_rx[o][i][n]=dan_rx[0]*cos(robot_yaw)-dan_ry[0]*sin(robot_yaw)+robot_x;
 mark_ry[o][i][n]=dan_rx[0]*sin(robot_yaw)+dan_ry[0]*cos(robot_yaw)+robot_y;
 
for(i=1;i<50;i++){//5cmから2.5mまで円を5cm刻みで作る。
   for(n=0;n<8*i;n++){
 mark_rx[o][i][n]=mark_rx[0][0][0]+0.05*i*cos(2*M_PI/8*i);
 mark_ry[o][i][n]=mark_ry[0][0][0]+0.05*i*sin(2*M_PI/8*i);
 }
}
}

//左危険領域のコスト変更範囲の決定
for(o=0; o<lo; o++){//オクルージョンの数だけ試行する
 //中心
 mark_lx[o][i][n]=dan_lx[0]*cos(robot_yaw)-dan_ly[0]*sin(robot_yaw)+robot_x;
 mark_ly[o][i][n]=dan_lx[0]*sin(robot_yaw)+dan_ly[0]*cos(robot_yaw)+robot_y;
 
for(i=1;i<50;i++){//5cmから2.5mまで円を5cm刻みで作る。
   for(n=0;n<8*i;n++){
 mark_lx[o][i][n]=mark_lx[0][0][0]+0.05*i*cos(2*M_PI/8*i);
 mark_ly[o][i][n]=mark_ly[0][0][0]+0.05*i*sin(2*M_PI/8*i);
 }
}
}

//範囲の決定(要改善)
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
int o=0;//オクルージョンの数
int i=0;//何番目の円か
int n=0;//一周のうち、何番目の点か
  unsigned int mrx[10][50][500];
  unsigned int mry[10][50][500];
  unsigned int mlx[10][50][500];
  unsigned int mly[10][50][500];

//右のコスト
for(o=0; o<ro; o++){
  for(i=0;i<50;i++){
   for(n=0;n<8*i;n++){
  if(master_grid.worldToMap(mark_rx[o][i][n], mark_ry[o][i][n], mrx[o][i][n], mry[o][i][n])){
    master_grid.setCost( mrx[o][i][n], mry[o][i][n], -162*sqrt(0.05*i)+LETHAL_OBSTACLE);
  }
}}}
//左のコスト
for(o=0; o<lo; o++){
  for(i=0;i<50;i++){
   for(n=0;n<8*i;n++){
  if(master_grid.worldToMap(mark_lx[o][i][n], mark_ly[o][i][n], mlx[o][i][n], mly[o][i][n])){
    master_grid.setCost(mlx[o][i][n], mly[o][i][n], -162*sqrt(0.05*i)+LETHAL_OBSTACLE);
  }
}}}
}

} // end namespace
