#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include <robot_communication/localizationInfoBroadcast.h>

#include <cmath>
#include <mutex>

static double wrapToPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

class DiffPurePursuit {
public:
  DiffPurePursuit(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {

    // ---- params ----
    pnh_.param<std::string>("topic_path", topic_path_, "/opt_path");
    pnh_.param<std::string>("topic_odom", topic_odom_, "/truth_pose_odom");
    pnh_.param<std::string>("topic_cmd",  topic_cmd_,  "/chassis_control");

    pnh_.param<double>("lookahead_dist", lookahead_dist_, 0.8);    // 追踪距离（米）
    pnh_.param<double>("goal_tolerance", goal_tolerance_, 0.25);   // 末端容差
    pnh_.param<double>("max_v", max_v_, 0.6);
    pnh_.param<double>("max_w", max_w_, 1.2);
    pnh_.param<double>("min_v", min_v_, 0.05); // 避免太小不动
    pnh_.param<double>("k_heading", k_heading_, 1.8); // 航向误差增益（角速度）
    pnh_.param<double>("v_scale", v_scale_, 0.8);     // 速度比例（跟误差/曲率相关）
    pnh_.param<double>("cmd_rate", cmd_rate_, 30.0);
    pnh_.param<double>("timeout_path", timeout_path_, 1.0);
    pnh_.param<double>("timeout_odom", timeout_odom_, 0.5);

    // ---- pubs/subs ----
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_cmd_, 1);

    path_sub_ = nh_.subscribe(topic_path_, 1, &DiffPurePursuit::pathCb, this);
    odom_sub_ = nh_.subscribe(topic_odom_, 1, &DiffPurePursuit::odomCb, this);

    timer_ = nh_.createTimer(ros::Duration(1.0 / cmd_rate_), &DiffPurePursuit::onTimer, this);

    ROS_INFO("[DiffPurePursuit] start. path=%s odom=%s cmd=%s",
             topic_path_.c_str(), topic_odom_.c_str(), topic_cmd_.c_str());
  }

private:
  void pathCb(const nav_msgs::PathConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    path_ = *msg;
    last_path_time_ = ros::Time::now();
  }

  void odomCb(const robot_communication::localizationInfoBroadcastConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);

    // 注意：字段名如果与你的 msg 不一致，你只要改这几行就行
    x_ = msg->xPosition;
    y_ = msg->yPosition;
    yaw_ = msg->chassisAngle;  // 如果你的是角度单位不是弧度，需要你这里转换

    vx_meas_ = msg->xSpeed;
    wz_meas_ = msg->chassisGyro;

    last_odom_time_ = ros::Time::now();
  }

  bool pickLookaheadPoint(double& tx, double& ty) {
    // 在全局坐标系下，从路径中找距离当前位置 lookahead_dist 的点
    if (path_.poses.empty()) return false;

    // 从最近点开始往前累计距离
    int nearest_i = 0;
    double best_d2 = 1e100;
    for (int i = 0; i < (int)path_.poses.size(); ++i) {
      double px = path_.poses[i].pose.position.x;
      double py = path_.poses[i].pose.position.y;
      double d2 = (px - x_) * (px - x_) + (py - y_) * (py - y_);
      if (d2 < best_d2) {
        best_d2 = d2;
        nearest_i = i;
      }
    }

    // 从 nearest_i 开始找第一个距离 >= lookahead_dist 的点
    double accum = 0.0;
    double prev_x = path_.poses[nearest_i].pose.position.x;
    double prev_y = path_.poses[nearest_i].pose.position.y;

    for (int i = nearest_i + 1; i < (int)path_.poses.size(); ++i) {
      double px = path_.poses[i].pose.position.x;
      double py = path_.poses[i].pose.position.y;
      double ds = std::hypot(px - prev_x, py - prev_y);
      accum += ds;
      prev_x = px; prev_y = py;

      if (accum >= lookahead_dist_) {
        tx = px; ty = py;
        return true;
      }
    }

    // 如果路径太短，就用末端点  测试一下
    tx = path_.poses.back().pose.position.x;
    ty = path_.poses.back().pose.position.y;
    return true;
  }

  void stop() {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_.publish(cmd);
  }

  void onTimer(const ros::TimerEvent&) {
    nav_msgs::Path path_copy;
    ros::Time t_path, t_odom;
    double x, y, yaw;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      path_copy = path_;
      t_path = last_path_time_;
      t_odom = last_odom_time_;
      x = x_; y = y_; yaw = yaw_;
    }

    const ros::Time now = ros::Time::now();
    if ((now - t_odom).toSec() > timeout_odom_) {
      ROS_WARN_THROTTLE(1.0, "[DiffPurePursuit] odom timeout -> stop");
      stop();
      return;
    }
    if ((now - t_path).toSec() > timeout_path_ || path_copy.poses.empty()) {
      ROS_WARN_THROTTLE(1.0, "[DiffPurePursuit] path timeout/empty -> stop");
      stop();
      return;
    }

    // 选追踪点
    double tx, ty;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      // 使用内部 path_ / x_ y_ yaw_
      if (!pickLookaheadPoint(tx, ty)) {
        stop();
        return;
      }
    }

    // 判断是否到达末端
    double gx = path_copy.poses.back().pose.position.x;
    double gy = path_copy.poses.back().pose.position.y;
    double dist_to_goal = std::hypot(gx - x, gy - y);
    if (dist_to_goal < goal_tolerance_) {
      ROS_INFO_THROTTLE(1.0, "[DiffPurePursuit] goal reached -> stop");
      stop();
      return;
    }

    // 计算追踪点在机器人坐标系下的相对位置
    // global -> local (base_link): rotate by -yaw
    double dx = tx - x;
    double dy = ty - y;
    double c = std::cos(-yaw);
    double s = std::sin(-yaw);
    double lx = c * dx - s * dy;
    double ly = s * dx + c * dy;

    // 追踪点如果在车后方，容易抖动：直接“就近转向”
    if (lx < 0.05) {
      lx = 0.05;
    }

    double heading_err = std::atan2(ly, lx);       // [-pi, pi]
    heading_err = wrapToPi(heading_err);

    // 差速控制：wz 由航向误差决定
    double wz = k_heading_ * heading_err;
    wz = clamp(wz, -max_w_, max_w_);

    // vx：跟航向误差大小反比（误差越大越慢），并限幅
    double v = max_v_ * v_scale_ * std::cos(heading_err);
    v = clamp(v, 0.0, max_v_);
    if (v > 0.0 && v < min_v_) v = min_v_;

    geometry_msgs::Twist cmd;
    cmd.linear.x = v;
    cmd.linear.y = 0.0;
    cmd.angular.z = wz;

    cmd_pub_.publish(cmd);
  }

private:
  ros::NodeHandle nh_, pnh_;

  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  cmd_pub_;
  ros::Timer timer_;

  std::mutex mtx_;

  nav_msgs::Path path_;
  ros::Time last_path_time_{0};
  ros::Time last_odom_time_{0};

  // pose
  double x_{0}, y_{0}, yaw_{0};
  // measured (not used now, reserved)
  double vx_meas_{0}, wz_meas_{0};

  // params
  std::string topic_path_, topic_odom_, topic_cmd_;
  double lookahead_dist_{0.8};
  double goal_tolerance_{0.25};
  double max_v_{0.6}, max_w_{1.2}, min_v_{0.05};
  double k_heading_{1.8};
  double v_scale_{0.8};
  double cmd_rate_{30.0};
  double timeout_path_{1.0}, timeout_odom_{0.5};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "diff_pure_pursuit_local_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  DiffPurePursuit node(nh, pnh);
  ros::spin();
  return 0;
}
