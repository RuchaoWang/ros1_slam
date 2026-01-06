#include "local_plan/differential_dwa.h"

Differential_DWAPlanner::Differential_DWAPlanner(void)
    : local_nh("~"), local_goal_subscribed(false), scan_updated(false), local_map_updated(false), odom_updated(false)
{
  local_nh.param("HZ", HZ, {20});
  local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
  local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
  local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.0});
  local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
  local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
  local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
  local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
  local_nh.param("MAX_DIST", MAX_DIST, {10.0});
  local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, {0.1});
  local_nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, {0.1});
  local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
  local_nh.param("PREDICT_TIME", PREDICT_TIME, {3.0});
  local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {1.0});
  local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
  local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
  local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
  local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
  DT = 1.0 / HZ;
  
  // 跟随路径集合的发布
  candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  // 选定的跟随路径
  selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  // 发布局部规划路径
  localPathPub = local_nh.advertise<nav_msgs::Path>("/local_path",1);
  // 发布局部规划出来的速度
  chassCtlPub = local_nh.advertise<geometry_msgs::Twist>("/chassis_control", 1, true);

  local_goal_sub = nh.subscribe("/local_goal", 1, &Differential_DWAPlanner::local_goal_callback, this);
  local_map_sub = nh.subscribe("/local_map_inflate", 1, &Differential_DWAPlanner::local_map_callback, this);
  odom_sub = nh.subscribe("/truthPose", 1, &Differential_DWAPlanner::odom_callback, this);
  target_velocity_sub = nh.subscribe("/velocity_control", 1, &Differential_DWAPlanner::target_velocity_callback, this);
}

Differential_DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _yawrate)
    : x(_x), y(_y), yaw(_yaw), velocity(_velocity), yawrate(_yawrate)
{
}

Differential_DWAPlanner::Window::Window(void)
    : min_velocity(0.0), max_velocity(0.0), min_yawrate(0.0), max_yawrate(0.0)
{
}

Differential_DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    : min_velocity(min_v), max_velocity(max_v), min_yawrate(min_y), max_yawrate(max_y)
{
}

void Differential_DWAPlanner::local_goal_callback(const robot_communication::goalConstPtr &msg)
{
  local_goal = *msg;
  local_goal_subscribed = true;
}

void Differential_DWAPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  local_map = *msg;
  local_map_updated = true;
}

void Differential_DWAPlanner::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  current_velocity = msg->twist.twist;

  // 当前速度过小的时候，提供一个激励
  // 因为是全向移动底盘，计算一个合速度
  double velocity = sqrt(pow(current_velocity.linear.x,2)+pow(current_velocity.linear.y,2));
  if(abs(velocity) <= 0.2)
  {
    velocity = 0.2;
  }
  current_velocity.linear.x = velocity;

  odom_updated = true;
}

void Differential_DWAPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  // 目标速度也需要合成
  TARGET_VELOCITY = sqrt(pow(msg->linear.x,2)+pow(msg->linear.y,2));
  ROS_INFO_STREAM("target velocity was updated to " << TARGET_VELOCITY << "[m/s]");
}

std::vector<Differential_DWAPlanner::State> Differential_DWAPlanner::dwa_planning(
    Window dynamic_window,
    Eigen::Vector3d goal,
    std::vector<std::vector<float>> obs_list)
{
  float min_cost = 1e6;
  float min_obs_cost = min_cost;
  float min_goal_cost = min_cost;
  float min_speed_cost = min_cost;

  std::vector<std::vector<State>> trajectories;
  std::vector<State> best_traj;

  // 对速度上进行采样
  for (float v = dynamic_window.min_velocity; v <= dynamic_window.max_velocity; v += VELOCITY_RESOLUTION)
  {
    // 对角速度进行采样
    for (float y = dynamic_window.min_yawrate; y <= dynamic_window.max_yawrate; y += YAWRATE_RESOLUTION)
    {
      State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
      std::vector<State> traj;
      // 对速度和角速度积分预测机器人位置
      for (float t = 0; t <= PREDICT_TIME; t += DT)
      {
        motion(state, v, y);
        // 将预测状态添加进来
        traj.push_back(state);
      }
      trajectories.push_back(traj);
      
      // 计算到终点的代价
      float to_goal_cost = calc_to_goal_cost(traj, goal);
      // 计算速度代价
      float speed_cost = calc_speed_cost(traj, TARGET_VELOCITY);
      // 计算障碍物代价
      float obstacle_cost = calc_obstacle_cost(traj, obs_list);
      // 计算最终总的代价，这里每*系数代表各项安全性指标  可以创新
      float final_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost + OBSTACLE_COST_GAIN * obstacle_cost;

      // 筛选出来最小的放在这
      if (min_cost >= final_cost)
      {
        min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost;
        min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost;
        min_speed_cost = SPEED_COST_GAIN * speed_cost;
        min_cost = final_cost;
        best_traj = traj;
      }
    }
  }
  ROS_INFO_STREAM("Cost: " << min_cost);
  ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
  ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
  ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
  ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
  visualize_trajectories(trajectories, 0, 1, 0, 100, candidate_trajectories_pub);
  if (min_cost == 1e6)
  {
    std::vector<State> traj;
    State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
    traj.push_back(state);
    best_traj = traj;
  }
  return best_traj;
}

void Differential_DWAPlanner::process(void)
{
  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    ROS_INFO("==========================================");
    double start = ros::Time::now().toSec();
    if (local_map_updated && local_goal_subscribed && odom_updated)
    {
      Window dynamic_window = calc_dynamic_window(current_velocity);
      Eigen::Vector3d goal(local_goal.Position_x, local_goal.Position_y,local_goal.Position_yaw/180*M_PI);
      ROS_INFO_STREAM("local goal: (" << goal[0] << "," << goal[1] << "," << goal[2] / M_PI * 180 << ")");
      geometry_msgs::Twist cmd_vel;
      if (goal.segment(0, 2).norm() > GOAL_THRESHOLD)
      {
        cout<<"1 goal.segment(0, 2).norm():"<<goal.segment(0, 2).norm()<<endl;
        std::vector<std::vector<float>> obs_list;
        obs_list = raycast();
        local_map_updated = false;

        std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);

        cmd_vel.linear.x = best_traj[0].velocity;
        cmd_vel.angular.z = best_traj[0].yawrate;
        visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
      }
      else
      {
        cout<<"2 goal.segment(0, 2).norm():"<<goal.segment(0, 2).norm()<<endl;
        cmd_vel.linear.x = 0.0;
        if (fabs(goal[2]) > TURN_DIRECTION_THRESHOLD)
        {
          cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
        }
        else
        {
          cmd_vel.angular.z = 0.0;
        }
      }
      ROS_INFO_STREAM("cmd_vel: (" << cmd_vel.linear.x << "[m/s], " << cmd_vel.angular.z << "[rad/s])");
      chassCtlPub.publish(cmd_vel);

      odom_updated = false;
    }
    else
    {
      std::cout<<"local_map_updated:"<<local_map_updated<<std::endl;
      if (!local_goal_subscribed)
      {
        ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
      }
      if (!odom_updated)
      {
        ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
      }
      if (!local_map_updated)
      {
        ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO_STREAM("loop time: " << ros::Time::now().toSec() - start << "[s]");
  }
}

Differential_DWAPlanner::Window Differential_DWAPlanner::calc_dynamic_window(const geometry_msgs::Twist &cur_velocity)
{
  Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_YAWRATE, MAX_YAWRATE);
  window.min_velocity = std::max((current_velocity.linear.x - MAX_ACCELERATION * DT), MIN_VELOCITY);
  window.max_velocity = std::min((current_velocity.linear.x + MAX_ACCELERATION * DT), MAX_VELOCITY);
  window.min_yawrate = std::max((current_velocity.angular.z - MAX_D_YAWRATE * DT), -MAX_YAWRATE);
  window.max_yawrate = std::min((current_velocity.angular.z + MAX_D_YAWRATE * DT), MAX_YAWRATE);
  return window;
}

float Differential_DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
  Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

// 计算速度代价数值
float Differential_DWAPlanner::calc_speed_cost(const std::vector<State> &traj, const float target_velocity)
{
  float cost = fabs(target_velocity - fabs(traj[traj.size() - 1].velocity));
  return cost;
}

// 计算到障碍物的距离数值
float Differential_DWAPlanner::calc_obstacle_cost(std::vector<State> &traj,const std::vector<std::vector<float>> &obs_list)
{
  float cost = 0.0;
  float min_dist = 1e3;
  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list)
    {
      float dist = sqrt((state.x - obs[0]) * (state.x - obs[0]) + (state.y - obs[1]) * (state.y - obs[1]));
      if (dist <= local_map.info.resolution)
      {
        cost = 1e6;
        return cost;
      }
      min_dist = std::min(min_dist, dist);
    }
  }
  cost = 1.0 / min_dist;
  return cost;
}

// 机器人的运动模型
void Differential_DWAPlanner::motion(State &state, const double velocity, const double yawrate)
{
  state.yaw += yawrate * DT;
  state.x += velocity * std::cos(state.yaw) * DT;
  state.y += velocity * std::sin(state.yaw) * DT;
  state.velocity = velocity;
  state.yawrate = yawrate;
}

// 通过激光雷达获取障碍物数据
std::vector<std::vector<float>> Differential_DWAPlanner::scan_to_obs()
{
  std::vector<std::vector<float>> obs_list;
  // 获取雷达扫描范围的最小角度
  float angle = scan.angle_min;
  // 进行最小范围的扫描
  for (auto r : scan.ranges)
  {
    // 获取障碍物的点集合
    float x = r * cos(angle);
    float y = r * sin(angle);
    std::vector<float> obs_state = {x, y};
    obs_list.push_back(obs_state);
    angle += scan.angle_increment;
  }
  return obs_list;
}


std::vector<std::vector<float>> Differential_DWAPlanner::raycast()
{
  std::vector<std::vector<float>> obs_list;
  for (float angle = -M_PI; angle <= M_PI; angle += ANGLE_RESOLUTION)
  {
    for (float dist = 0.0; dist <= MAX_DIST; dist += local_map.info.resolution)
    {
      float x = dist * cos(angle);
      float y = dist * sin(angle);
      int i = floor(x / local_map.info.resolution + 0.5) + local_map.info.width * 0.5;
      int j = floor(y / local_map.info.resolution + 0.5) + local_map.info.height * 0.5;
      if ((i < 0 || i >= local_map.info.width) || (j < 0 || j >= local_map.info.height))
      {
        break;
      }
      if (local_map.data[j * local_map.info.width + i] == 100)
      {
        std::vector<float> obs_state = {x, y};
        obs_list.push_back(obs_state);
        break;
      }
    }
  }
  return obs_list;
}

void Differential_DWAPlanner::visualize_trajectories(const std::vector<std::vector<State>> &trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  int count = 0;
  const int size = trajectories.size();
  for (; count < size; count++)
  {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectory.scale.x = 0.02;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    for (const auto &pose : trajectories[count])
    {
      p.x = pose.x;
      p.y = pose.y;
      v_trajectory.points.push_back(p);
    }
    v_trajectories.markers.push_back(v_trajectory);
  }
  for (; count < trajectories_size;)
  {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::DELETE;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectories.markers.push_back(v_trajectory);
    count++;
  }
  pub.publish(v_trajectories);
}

void Differential_DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory, const double r, const double g, const double b, const ros::Publisher &pub)
{
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = ROBOT_FRAME;
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r;
  v_trajectory.color.g = g;
  v_trajectory.color.b = b;
  v_trajectory.color.a = 0.8;
  v_trajectory.ns = pub.getTopic();
  v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.scale.x = 0.05;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  v_trajectory.pose = pose;
  geometry_msgs::Point p;
  geometry_msgs::PoseStamped point;
  // 发布局部规划路径
  nav_msgs::Path local_path;
  // 发布局部规划的路径
  for (const auto &pose : trajectory)
  {
    p.x = pose.x;
    p.y = pose.y;
    v_trajectory.points.push_back(p);

    point.pose.position.x = pose.x;
    point.pose.position.y = pose.y;
    local_path.poses.push_back(point);
  }

  // 发布跟随路径
  pub.publish(v_trajectory);

  // 汽车基坐标系
  local_path.header.frame_id = ROBOT_FRAME;
  // 发布时间戳
  local_path.header.stamp = ros::Time::now();
  // 发布跟随以nav_msgs::Path的消息类型发布路径
  localPathPub.publish(local_path);
}
