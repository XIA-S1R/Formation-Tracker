#include "maps.hpp"

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "perlinnoise.hpp"

using namespace mocka;

// @GaaiLam
void Maps::addGround() {
  int add_ground;
  info.nh_private->param("ground", add_ground, 0);
  if (add_ground) {
    double _x_l = -info.sizeX / (2 * info.scale);
    double _y_l = -info.sizeY / (2 * info.scale);
    for (int i=0; i<info.sizeX; ++i) {
      for (int j=0; j<info.sizeY; ++j) {
        info.cloud->points.emplace_back(
          _x_l+i*1.0/info.scale, _y_l+j*1.0/info.scale, 0);
      }
    }
  }
}

// 生成矩形障碍物
void Maps::generateBox(double x, double y, double z, double width, double length, double height, double resolution, pcl::PointXYZ& pt)
{
  int widNum = ceil(width / resolution);
  int lenNum = ceil(length / resolution);
  int heiNum = ceil(height / resolution);

  int rl, rh, sl, sh;
  rl = -widNum / 2;
  rh = widNum / 2;
  sl = -lenNum / 2;
  sh = lenNum / 2;

  // 确保z方向位置合理，使矩形底部接触地面
  double base_z = 0.0; // 地面高度

  for (int r = rl; r < rh; r++)
    for (int s = sl; s < sh; s++)
    {
      for (int t = 0; t < heiNum; t++)
      {
        if ((r - rl) * (r - rh + 1) * (s - sl) * (s - sh + 1) * t *
              (t - heiNum + 1) ==
            0)
        {
          pt.x = x + r * resolution;
          pt.y = y + s * resolution;
          pt.z = base_z + t * resolution;
          info.cloud->points.push_back(pt);
        }
      }
    }
}

// 基于swarm_formation的map_generator包中的random_forest_sensing.cpp实现
void
Maps::randomMapGenerate()
{
  std::default_random_engine eng(info.seed);
  double _resolution = 1 / info.scale;

  double _x_l = -info.sizeX / (2 * info.scale);
  double _x_h = info.sizeX / (2 * info.scale);
  double _y_l = -info.sizeY / (2 * info.scale);
  double _y_h = info.sizeY / (2 * info.scale);
  
  // 计算地图的实际尺寸（以米为单位）
  double map_width_x = info.sizeX / info.scale; // x轴总长度
  double map_width_y = info.sizeY / info.scale; // y轴总长度
  double map_width_z = info.sizeZ / info.scale; // z轴总长度
  
  // 计算x方向的区域划分（考虑地图中心在原点）
  double left_third = map_width_x / 6.0; // 左侧1/6区域长度
  double middle_third = map_width_x * 2.0 / 3.0; // 中间2/3区域长度
  double right_third = map_width_x / 3.0; // 右侧1/3区域长度
  
  // 计算各区域的边界
  double left_bound = -map_width_x / 2.0 + left_third; // 左侧1/6结束的位置
  double right_bound = map_width_x / 2.0 - right_third; // 右侧1/3开始的位置
  double right_start = right_bound; // 右侧1/3区域的起始位置
  double right_end = map_width_x / 2.0; // 右侧1/3区域的结束位置
  
  ROS_INFO("Map dimensions: x=%.2f, y=%.2f, z=%.2f", 
           map_width_x, map_width_y, map_width_z);
  ROS_INFO("X ranges: left_bound=%.2f, right_bound=%.2f", left_bound, right_bound);
  ROS_INFO("Right third: start=%.2f, end=%.2f", right_start, right_end);

  // 从参数服务器获取配置
  double _w_l, _w_h, _h_l, _h_h;
  int    _obs_num, circle_num_;
  double radius_l_, radius_h_, z_l_, z_h_, theta_, _min_dist;

  info.nh_private->param("width_min", _w_l, 0.2);
  info.nh_private->param("width_max", _w_h, 1.5);
  info.nh_private->param("height_min", _h_l, 2.0);
  info.nh_private->param("height_max", _h_h, 10.0);
  info.nh_private->param("obstacle_number", _obs_num, 30);
  info.nh_private->param("circle_number", circle_num_, 30);
  info.nh_private->param("radius_min", radius_l_, 0.5);
  info.nh_private->param("radius_max", radius_h_, 2.0);
  info.nh_private->param("z_min", z_l_, 0.7);
  info.nh_private->param("z_max", z_h_, 3.0);
  info.nh_private->param("theta", theta_, 0.5);
  info.nh_private->param("min_distance", _min_dist, 0.8);

  // 确保参数合理
  _h_l = _h_l >= 0 ? _h_l : 0;
  _h_h = _h_h <= map_width_z ? _h_h : map_width_z;

  // 初始化随机分布，确保障碍物只生成在中间的2/3区域
  std::uniform_real_distribution<double> rand_x(left_bound, right_bound); // 确保障碍物位于x方向的有效范围内
  std::uniform_real_distribution<double> rand_y(_y_l, _y_h);
  std::uniform_real_distribution<double> rand_w(_w_l, _w_h);
  std::uniform_real_distribution<double> rand_h(_h_l, _h_h);
  std::uniform_real_distribution<double> rand_inf(0.5, 1.5);
  std::uniform_real_distribution<double> rand_radius_(radius_l_, radius_h_);
  std::uniform_real_distribution<double> rand_radius2_(radius_l_, 1.2);
  std::uniform_real_distribution<double> rand_theta_(-theta_, theta_);
  std::uniform_real_distribution<double> rand_z_(z_l_, z_h_);

  pcl::PointXYZ pt_random;
  std::vector<Eigen::Vector2d> obs_position;

  // 生成圆柱形障碍物
  for (int i = 0; i < _obs_num && ros::ok(); i++)
  {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    // 检查障碍物间距
    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    // 调整坐标到网格点
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w * inf) / _resolution);
    double radius = (w * inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = 0; t < heiNum; t++) // 从地面开始生成
        {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = t * _resolution + 1e-2; // 从地面开始
          
          // 只生成圆柱表面的点
          if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            info.cloud->points.push_back(pt_random);
          }
        }
      }
  }

  // 生成环形障碍物
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    // 调整坐标到网格点
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    // 确保环的底部接触地面
    double base_z = 0.0;
    z = base_z; // 固定z坐标为地面高度

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

    double radius1 = rand_radius_(eng);
    double radius2 = radius1; // 确保环是正圆，不是椭圆
    // 确保环有足够的高度，避免变成扁平的椭圆
    double min_height = 0.5; // 最小高度
    if (radius2 < min_height) {
      radius2 = min_height;
    }

    // 生成完整的环形
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // 生成点
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution, ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            info.cloud->points.push_back(pt_random);
          }
    }
  }

  // 计算地图的实际边界（考虑地图中心在原点）
  double map_min_x = -map_width_x / 2.0;
  double map_max_x = map_width_x / 2.0;
  double map_min_y = -map_width_y / 2.0;
  double map_max_y = map_width_y / 2.0;
  
  ROS_INFO("Map actual bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]", 
           map_min_x, map_max_x, map_min_y, map_max_y);
  
  // 计算x轴后1/3区域的长度
  double right_third_length = right_end - right_start;
  
  // 在x方向的后1/3区域添加大障碍物
  double big_obstacle_width = right_third_length * 0.4; // 缩短宽度，占后1/3区域的40%
  double big_obstacle_length = 10.0;
  double big_obstacle_height = 8.0;
  
  // 计算大障碍物的位置，确保整个障碍在后1/3区域内
  double big_obstacle_x_min = right_start;
  double big_obstacle_x_max = right_start + right_third_length * 0.45; // 为矩形障碍留出空间
  double big_obstacle_y_min = map_min_y + big_obstacle_length / 2.0;
  double big_obstacle_y_max = map_max_y - big_obstacle_length / 2.0;
  
  // 确保大障碍物的尺寸合理
  double actual_big_width = std::min(big_obstacle_width, big_obstacle_x_max - big_obstacle_x_min);
  double actual_big_length = std::min(big_obstacle_length, big_obstacle_y_max - big_obstacle_y_min);
  
  // 计算大障碍物的中心位置
  double big_obstacle_x = (big_obstacle_x_min + big_obstacle_x_max) / 2.0;
  double big_obstacle_y = 0.0;
  double big_obstacle_z = 0.0;
  
  generateBox(big_obstacle_x, big_obstacle_y, big_obstacle_z, 
              actual_big_width, actual_big_length, big_obstacle_height, 
              _resolution, pt_random);
  
  // 计算大障碍物的实际右边界
  double big_obstacle_right = big_obstacle_x + actual_big_width / 2.0;
  
  // 在大障碍物之后添加几列沿x轴方向的长矩形障碍
  int num_columns = 5; // 增加数量
  double column_width = right_third_length * 0.5; // 缩短宽度，占后1/3区域的50%
  double column_length = 1.0;
  double column_height = 6.0;
  double column_spacing = 3.0; // 减小间距
  double column_gap = 3.0; // 与大障碍物的间距，适当减小
  
  // 计算矩形障碍的起始x位置（在大障碍物之后，确保在后1/3区域内）
  double column_x_min = big_obstacle_right + column_gap;
  double column_x_max = right_end;
  double column_start_x = column_x_min + column_width / 2.0;
  
  // 确保矩形障碍的位置在后1/3区域内
  if (column_start_x + column_width / 2.0 > column_x_max) {
    column_start_x = column_x_max - column_width / 2.0;
  }
  
  for (int i = 0; i < num_columns; i++) {
    // 计算矩形障碍的x位置（确保在大障碍物之后）
    double column_x = column_start_x;
    
    double column_y = -column_spacing * (num_columns - 1) / 2.0 + i * column_spacing;
    // 确保整个矩形障碍在y轴边界内
    double column_y_min = map_min_y + column_length / 2.0;
    double column_y_max = map_max_y - column_length / 2.0;
    column_y = std::max(column_y, column_y_min);
    column_y = std::min(column_y, column_y_max);
    
    double column_z = 0.0;
    generateBox(column_x, column_y, column_z, 
                column_width, column_length, column_height, 
                _resolution, pt_random);
  }
  
  addGround();
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

void
Maps::pcl2ros()
{
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
  ROS_INFO("finish: infill %lf%%",
           info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
}

void
Maps::perlin3D()
{
  double complexity;
  double fill;
  int    fractal;
  double attenuation;

  info.nh_private->param("complexity", complexity, 0.142857);
  info.nh_private->param("fill", fill, 0.38);
  info.nh_private->param("fractal", fractal, 1);
  info.nh_private->param("attenuation", attenuation, 0.5);

  info.cloud->width  = info.sizeX * info.sizeY * info.sizeZ;
  info.cloud->height = 1;
  info.cloud->points.resize(info.cloud->width * info.cloud->height);

  PerlinNoise noise(info.seed);

  std::vector<double>* v = new std::vector<double>;
  v->reserve(info.cloud->width);
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        v->push_back(tnoise);
      }
    }
  }
  std::sort(v->begin(), v->end());
  int    tpos = info.cloud->width * (1 - fill);
  double tmp  = v->at(tpos);
  ROS_INFO("threshold: %lf", tmp);

  int pos = 0;
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        if (tnoise > tmp)
        {
          info.cloud->points[pos].x =
            i / info.scale - info.sizeX / (2 * info.scale);
          info.cloud->points[pos].y =
            j / info.scale - info.sizeY / (2 * info.scale);
          info.cloud->points[pos].z = k / info.scale;
          pos++;
        }
      }
    }
  }
  info.cloud->width = pos;
  ROS_INFO("the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  addGround();
  info.cloud->width = info.cloud->points.size();
  pcl2ros();
}

void
Maps::recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi& maze)
{
  ROS_INFO(
    "generating maze with width %d , height %d", xh - xl + 1, yh - yl + 1);

  if (xl < xh - 3 && yl < yh - 3)
  { // the remaining area is larger than or equal to 5*5, need to add both x
    // wall and y wall
    bool valid = false; // used to judge whether the wall selection is valid
    int  xm    = 0;
    int  ym    = 0;
    ROS_INFO("entered 5*5 mode");
    while (valid == false)
    {
      xm = (std::rand() % (xh - xl - 1) + xl +
            1); // generating random number between xl+1 and xh-1(pointless to
                // add a wall at the sides)
      ym = (std::rand() % (yh - yl - 1) + yl +
            1); // generating random number between yl+1 and yh-1(pointless to
                // add a wall at the sides)
      if (xl - 1 >= 0)
      { // there is a point at xl-1,ym
        if (maze(xl - 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (xh + 1 <= maze.cols() - 1)
      { // there is a point at xh+1,ym
        if (maze(xh + 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yl - 1 >= 0)
      { // there is a point at xm,yl-1
        if (maze(xm, yl - 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yh + 1 <= maze.rows() - 1)
      { // there is a point at xm,yh+1
        if (maze(xm, yh + 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      valid = true;

    } // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
      }
    }

    std::cout << maze << std::endl;
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze);

    ROS_INFO("finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  } // when the remaining area is larger than or equal to 5*5

  else if (xl < xh - 2 && yl < yh - 2)
  {
    bool valid     = false; // used to judge whether the wall selection is valid
    int  xm        = 0;
    int  ym        = 0;
    int  doorcount = 0;
    xm             = (std::rand() % (xh - xl - 1) + xl +
          1); // generating random number between xl+1 and xh-1(pointless to
                          // add a wall at the sides)
    ym =
      (std::rand() % (yh - yl - 1) + yl +
       1); // generating random number between yl+1 and yh-1(pointless to
           // add a wall at the sides)
           // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
        doorcount++;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
        doorcount++;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
        doorcount++;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
        doorcount++;
      }
    }

    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    std::cout << maze << std::endl;

    ROS_INFO("finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  }

  else if (xl < xh - 1 && yl < yh - 2)
  { // the case of 3*4+
    ROS_INFO("entered 3*4+ mode");
    int doorcount = 0;
    int ym        = 0;
    for (int i = yl; i <= yh; i++)
    {
      maze(xl + 1, i) = 1;
    } // filling a center wall
    if (yl - 1 >= 0)
    {
      if (maze(xl + 1, yl - 1) == 0)
      {
        maze(xl + 1, yl) = 0;
        doorcount++;
      }
    }
    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xl + 1, yh + 1) == 0)
      {
        maze(xl + 1, yh) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      ym               = std::rand() % (yh - yl + 1) + yl;
      maze(xl + 1, ym) = 0;
    }
  } // the case of 4+*3
  //
  else if (xl < xh - 2 && yl < yh - 1)
  { // the case of 4+*3
    ROS_INFO("entered 4+*3 mode");
    int doorcount = 0;
    int xm        = 0;
    for (int i = xl; i <= xh; i++)
    {
      maze(i, yl + 1) = 1;
    } // filling a center wall
    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, yl + 1) == 0)
      {
        maze(xl, yl + 1) = 0;
        doorcount++;
      }
    }
    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, yl + 1) == 0)
      {
        maze(xh, yl + 1) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      xm               = std::rand() % (xh - xl + 1) + xl;
      maze(xm, yl + 1) = 0;
    }
  } // the case of 4+*3

  else if (xl < xh - 1 && yl < yh - 1)
  { // the case of 3*3
    maze(xl + 1, yl + 1) = 1;
    return;
  }
  else
  {
    ROS_INFO("finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    return;
  }
}

void
Maps::recursizeDivisionMaze(Eigen::MatrixXi& maze)
{
  //! @todo all bugs here...
  int sx = maze.rows();
  int sy = maze.cols();

  int px, py;

  if (sx > 5)
    px = (std::rand() % (sx - 3) + 1);
  else
    return;

  if (sy > 5)
    py = (std::rand() % (sy - 3) + 1);
  else
    return;

  ROS_INFO("debug %d %d %d %d", sx, sy, px, py);

  int x1, x2, y1, y2;

  if (px != 1)
    x1 = (std::rand() % (px - 1) + 1);
  else
    x1 = 1;

  if ((sx - px - 3) > 0)
    x2 = (std::rand() % (sx - px - 3) + px + 1);
  else
    x2 = px + 1;

  if (py != 1)
    y1 = (std::rand() % (py - 1) + 1);
  else
    y1 = 1;

  if ((sy - py - 3) > 0)
    y2 = (std::rand() % (sy - py - 3) + py + 1);
  else
    y2 = py + 1;
  ROS_INFO("%d %d %d %d", x1, x2, y1, y2);

  if (px != 1 && px != (sx - 2))
  {
    for (int i = 1; i < (sy - 1); ++i)
    {
      if (i != y1 && i != y2)
        maze(px, i) = 1;
    }
  }
  if (py != 1 && py != (sy - 2))
  {
    for (int i = 1; i < (sx - 1); ++i)
    {
      if (i != x1 && i != x2)
        maze(i, py) = 1;
    }
  }
  switch (std::rand() % 4)
  {
    case 0:
      maze(x1, py) = 1;
      break;
    case 1:
      maze(x2, py) = 1;
      break;
    case 2:
      maze(px, y1) = 1;
      break;
    case 3:
      maze(px, y2) = 1;
      break;
  }

  if (px > 2 && py > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(0, 0, px, py) = sub;
  }
  if (px > 2 && (sy - py - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, py, px + 1, sy - py);
    recursizeDivisionMaze(sub);
    maze.block(0, py, px + 1, sy - py) = sub;
  }
  if (py > 2 && (sx - px - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(px, 0, sx - px, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(px, 0, sx - px, py + 1) = sub;
  }
  if ((sx - px - 1) > 2 && (sy - py - 1) > 2)
  {

    Eigen::MatrixXi sub = maze.block(px, py, sy - px, sy - py);

    recursizeDivisionMaze(sub);
    maze.block(px, py, sy - px, sy - py) = sub;
  }
}

void
Maps::maze2D()
{
  double width;
  int    type;
  int    addWallX;
  int    addWallY;
  info.nh_private->param("road_width", width, 1.0);
  info.nh_private->param("add_wall_x", addWallX, 0);
  info.nh_private->param("add_wall_y", addWallY, 0);
  info.nh_private->param("maze_type", type, 1);

  int mx = info.sizeX / (width * info.scale);
  int my = info.sizeY / (width * info.scale);

  Eigen::MatrixXi maze(mx, my);
  maze.setZero();

  switch (type)
  {
    case 1:
      recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);
      break;
  }

  if (addWallX)
  {
    for (int i = 0; i < mx; ++i)
    {
      maze(i, 0)      = 1;
      maze(i, my - 1) = 1;
    }
  }
  if (addWallY)
  {
    for (int i = 0; i < my; ++i)
    {
      maze(0, i)      = 1;
      maze(mx - 1, i) = 1;
    }
  }

  std::cout << maze << std::endl;

  for (int i = 0; i < mx; ++i)
  {
    for (int j = 0; j < my; ++j)
    {
      if (maze(i, j))
      {
        for (int ii = 0; ii < width * info.scale; ++ii)
        {
          for (int jj = 0; jj < width * info.scale; ++jj)
          {
            for (int k = 0; k < info.sizeZ; ++k)
            {
              pcl::PointXYZ pt_random;
              pt_random.x =
                i * width + ii / info.scale - info.sizeX / (2.0 * info.scale);
              pt_random.y =
                j * width + jj / info.scale - info.sizeY / (2.0 * info.scale);
              pt_random.z = k / info.scale;
              info.cloud->points.push_back(pt_random);
            }
          }
        }
      }
    }
  }
  addGround();
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

Maps::BasicInfo
Maps::getInfo() const
{
  return info;
}

void
Maps::setInfo(const BasicInfo& value)
{
  info = value;
}

Maps::Maps()
{
}

void
Maps::generate(int type)
{
  switch (type)
  {
    default:
    case 1:
      perlin3D();
      break;
    case 2:
      randomMapGenerate();
      break;
    case 3:
      std::srand(info.seed);
      maze2D();
      break;
    case 4: // generating 3d maze
      std::srand(info.seed);
      Maze3DGen();
      break;
  }
}

pcl::PointXYZ
MazePoint::getPoint()
{
  return point;
}

int
MazePoint::getPoint1()
{
  return point1;
}

int
MazePoint::getPoint2()
{
  return point2;
}

double
MazePoint::getDist1()
{
  return dist1;
}

double
MazePoint::getDist2()
{
  return dist2;
}

void
MazePoint::setPoint(pcl::PointXYZ p)
{
  point = p;
}

void
MazePoint::setPoint1(int p)
{
  point1 = p;
}

void
MazePoint::setPoint2(int p)
{
  point2 = p;
}

void
MazePoint::setDist1(double set)
{
  dist1 = set;
}

void
MazePoint::setDist2(double set)
{
  dist2 = set;
}

void
Maps::Maze3DGen()
{
  // getting required info parameters from the given node
  int    numNodes;
  double connectivity;
  int    nodeRad;
  int    roadRad;

  info.nh_private->param("numNodes", numNodes, 10);
  info.nh_private->param("connectivity", connectivity, 0.5);
  info.nh_private->param("nodeRad", nodeRad, 3);
  info.nh_private->param("roadRad", roadRad, 2);
  ROS_INFO("received parameters : numNodes: %d connectivity: "
           "%f nodeRad: %d roadRad: %d",
           numNodes,
           connectivity,
           nodeRad,
           roadRad);
  // generating random points
  std::vector<pcl::PointXYZ> base;

  for (int i = 0; i < numNodes; i++)
  {
    double rx = std::rand() / RAND_MAX +
                (std::rand() % info.sizeX) / info.scale -
                info.sizeX / (2 * info.scale);
    double ry = std::rand() / RAND_MAX +
                (std::rand() % info.sizeY) / info.scale -
                info.sizeY / (2 * info.scale);
    double rz = std::rand() / RAND_MAX +
                (std::rand() % info.sizeZ) / info.scale -
                info.sizeZ / (2 * info.scale);
    ROS_INFO("point: x: %f , y: %f , z: %f", rx, ry, rz);

    pcl::PointXYZ pt_random;
    pt_random.x = rx;
    pt_random.y = ry;
    pt_random.z = rz;
    base.push_back(pt_random);
  } // generating random cores in the space

  for (int i = 0; i < info.sizeX; i++)
  {
    for (int j = 0; j < info.sizeY; j++)
    {
      for (int k = 0; k < info.sizeZ; k++)
      { // for every scaled coordinate points
        pcl::PointXYZ test;
        test.x = i / info.scale - info.sizeX / (2 * info.scale);
        test.y = j / info.scale - info.sizeY / (2 * info.scale);
        test.z = k / info.scale -
                 info.sizeZ /
                   (2 * info.scale); // marking the corresponding point location

        MazePoint mp;
        mp.setPoint(test);
        mp.setPoint2(-1);
        mp.setPoint1(-1);
        mp.setDist1(10000.0);
        mp.setDist2(100000.0); // setting super large starting values
        for (int ii = 0; ii < numNodes; ii++)
        {
          double dist =
            std::sqrt((base[ii].x - test.x) * (base[ii].x - test.x) +
                      (base[ii].y - test.y) * (base[ii].y - test.y) +
                      (base[ii].z - test.z) * (base[ii].z - test.z));
          if (dist < mp.getDist1())
          {

            mp.setDist2(mp.getDist1());
            mp.setDist1(dist);

            mp.setPoint2(mp.getPoint1());
            mp.setPoint1(ii);
          }
          else if (dist < mp.getDist2())
          {
            mp.setDist2(dist);
            mp.setPoint2(ii);
          } // finding the distances to the nearest two cores
        }
        if (std::abs(mp.getDist2() - mp.getDist1()) < 1 / info.scale)
        { // the tested location is on one of the middle planes
          if ((mp.getPoint1() + mp.getPoint2()) >
                int((1 - connectivity) * numNodes) &&
              (mp.getPoint1() + mp.getPoint2()) <
                int((1 + connectivity) * numNodes))
          { // this is a holed wall
            double judge =
              std::sqrt((base[mp.getPoint1()].x - base[mp.getPoint2()].x) *
                          (base[mp.getPoint1()].x - base[mp.getPoint2()].x) +
                        (base[mp.getPoint1()].y - base[mp.getPoint2()].y) *
                          (base[mp.getPoint1()].y - base[mp.getPoint2()].y) +
                        (base[mp.getPoint1()].z - base[mp.getPoint2()].z) *
                          (base[mp.getPoint1()].z - base[mp.getPoint2()].z));
            if (mp.getDist1() + mp.getDist2() - judge >=
                roadRad / (info.scale * 3))
            {
              info.cloud->points.push_back(mp.getPoint());
            }
          }
          else
          {
            info.cloud->points.push_back(mp.getPoint());
          }
        }
      }
    }
  }

  info.cloud->width  = info.cloud->points.size();
  info.cloud->height = 1;
  ROS_INFO("the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}