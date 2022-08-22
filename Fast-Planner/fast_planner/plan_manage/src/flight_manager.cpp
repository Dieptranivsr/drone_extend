/*
    --- fast-planner-lite ---
*/

// #include <fstream>
#include <plan_manage/flight_manager.h>
#include <thread>

namespace fast_planner {

// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  org_path = node_.advertise<visualization_msgs::Marker>("/check/path_points", 20);
  pubs_spline.push_back(org_path);
  bspline_points = node_.advertise<visualization_msgs::Marker>("/bspline/path_points", 20);
  pubs_spline.push_back(bspline_points);
  control_points = node_.advertise<visualization_msgs::Marker>("/bspline/control_points", 20);
  pubs_spline.push_back(control_points);
  bspline_opt = node_.advertise<visualization_msgs::Marker>("/opt/bspline_points", 20);
  pubs_spline.push_back(bspline_opt);
  ctrl_point_opt = node_.advertise<visualization_msgs::Marker>("/opt/control_points", 20);
  pubs_spline.push_back(ctrl_point_opt);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  kino_path_finder_.reset(new KinodynamicAstar);
  kino_path_finder_->setParam(nh);
  kino_path_finder_->setEnvironment(edt_environment_);
  kino_path_finder_->init();

  bspline_optimizers_.resize(10);
  for (int i = 0; i < 10; ++i) {
    bspline_optimizers_[i].reset(new BsplineOptimizer);
    bspline_optimizers_[i]->setParam(nh);
    bspline_optimizers_[i]->setEnvironment(edt_environment_);
  }
}

bool FastPlannerManager::checkTrajCollision(double& distance) {

  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.7) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  local_data_.start_time_ = ros::Time::now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();

  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  // point_path
  visualization_msgs::Marker mk0;
  mk0.header.frame_id  = "world";
  mk0.header.stamp     = ros::Time::now();
  mk0.type             = visualization_msgs::Marker::SPHERE_LIST;
  mk0.action           = visualization_msgs::Marker::DELETE;
  mk0.id               = 200;
  pubs_spline[0].publish(mk0);

  mk0.action             = visualization_msgs::Marker::ADD;
  mk0.pose.orientation.x = 0.0;
  mk0.pose.orientation.y = 0.0;
  mk0.pose.orientation.z = 0.0;
  mk0.pose.orientation.w = 1.0;

  mk0.color.r = 0.5;
  mk0.color.g = 0.0;
  mk0.color.b = 1.0;
  mk0.color.a = 0.5;
  
  mk0.scale.x = 0.1;
  mk0.scale.y = 0.1;
  mk0.scale.z = 0.1;
  
  geometry_msgs::Point pt0;
  for (int i = 0; i < int(plan_data_.kino_path_.size()); i++) {
    pt0.x = plan_data_.kino_path_[i](0);
    pt0.y = plan_data_.kino_path_[i](1);
    pt0.z = plan_data_.kino_path_[i](2);
    mk0.points.push_back(pt0);
  }
  pubs_spline[0].publish(mk0);
  ros::Duration(0.001).sleep();

  t_search = (ros::Time::now() - t1).toSec();

  // parameterize the path to bspline

  double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  //NonUniformBspline init(ctrl_pts, 3, ts);
  NonUniformBspline positions = NonUniformBspline(ctrl_pts, 3, ts);
 
  // bspline trajectory optimization

  // add bspline curves
  vector<Eigen::Vector3d> point_bspline, cpt_bspline;
  double                  tm1, tmp1;
  positions.getTimeSpan(tm1, tmp1);
  for (double t = tm1; t < tmp1; t += 0.01 ) {
    Eigen::Vector3d pt1 = positions.evaluateDeBoor(t);
    point_bspline.push_back(pt1);
  }

  visualization_msgs::Marker mk1;
  mk1.header.frame_id  = "world";
  mk1.header.stamp     = ros::Time::now();
  mk1.type             = visualization_msgs::Marker::SPHERE_LIST;
  mk1.action           = visualization_msgs::Marker::DELETE;
  mk1.id               = 300;
  pubs_spline[1].publish(mk1);

  mk1.action             = visualization_msgs::Marker::ADD;
  mk1.pose.orientation.x = 0.0;
  mk1.pose.orientation.y = 0.0;
  mk1.pose.orientation.z = 0.0;
  mk1.pose.orientation.w = 1.0;

  mk1.color.r = 0.15;
  mk1.color.g = 1.0;
  mk1.color.b = 0.8;
  mk1.color.a = 0.8;
  
  mk1.scale.x = 0.1;
  mk1.scale.y = 0.1;
  mk1.scale.z = 0.1;
  
  geometry_msgs::Point pt1_;
  for (int i = 0; i < int(point_bspline.size()); i++) {
    pt1_.x = point_bspline[i](0);
    pt1_.y = point_bspline[i](1);
    pt1_.z = point_bspline[i](2);
    mk1.points.push_back(pt1_);
  }
  pubs_spline[1].publish(mk1);
  ros::Duration(0.001).sleep();

  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    Eigen::Vector3d pt2 = ctrl_pts.row(i).transpose();
    cpt_bspline.push_back(pt2);
  }

  visualization_msgs::Marker mk2;
  mk2.header.frame_id  = "world";
  mk2.header.stamp     = ros::Time::now();
  mk2.type             = visualization_msgs::Marker::SPHERE_LIST;
  mk2.action           = visualization_msgs::Marker::DELETE;
  mk2.id               = 400;
  pubs_spline[2].publish(mk2);

  mk2.action             = visualization_msgs::Marker::ADD;
  mk2.pose.orientation.x = 0.0;
  mk2.pose.orientation.y = 0.0;
  mk2.pose.orientation.z = 0.0;
  mk2.pose.orientation.w = 1.0;

  mk2.color.r = 1.0;
  mk2.color.g = 0.5;
  mk2.color.b = 1.0;
  mk2.color.a = 1.0;
  
  mk2.scale.x = 0.2;
  mk2.scale.y = 0.2;
  mk2.scale.z = 0.2;
  
  geometry_msgs::Point pt2;
  for (int i = 0; i < int(cpt_bspline.size()); i++) {
    pt2.x = cpt_bspline[i](0);
    pt2.y = cpt_bspline[i](1);
    pt2.z = cpt_bspline[i](2);
    mk2.points.push_back(pt2);
  }
  pubs_spline[2].publish(mk2);
  ros::Duration(0.001).sleep();
  
  t1 = ros::Time::now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  t_opt = (ros::Time::now() - t1).toSec();

  // iterative time adjustment

  t1                    = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  // add bspline curves
  vector<Eigen::Vector3d> point_set_opt, cpt_opt;
  double                  tm2, tmp2;
  pos.getTimeSpan(tm2, tmp2);
  for (double t = tm2; t < tmp2; t += 0.01 ) {
    Eigen::Vector3d pt3 = pos.evaluateDeBoor(t);
    point_set_opt.push_back(pt3);
  }

  visualization_msgs::Marker mk3;
  mk3.header.frame_id  = "world";
  mk3.header.stamp     = ros::Time::now();
  mk3.type             = visualization_msgs::Marker::SPHERE_LIST;
  mk3.action           = visualization_msgs::Marker::DELETE;
  mk3.id               = 300;
  pubs_spline[3].publish(mk3);

  mk3.action             = visualization_msgs::Marker::ADD;
  mk3.pose.orientation.x = 0.0;
  mk3.pose.orientation.y = 0.0;
  mk3.pose.orientation.z = 0.0;
  mk3.pose.orientation.w = 1.0;

  mk3.color.r = 1;
  mk3.color.g = 0.15;
  mk3.color.b = 0.9;
  mk3.color.a = 0.8;
  
  mk3.scale.x = 0.1;
  mk3.scale.y = 0.1;
  mk3.scale.z = 0.1;
  
  geometry_msgs::Point pt3;
  for (int i = 0; i < int(point_set_opt.size()); i++) {
    pt3.x = point_set_opt[i](0);
    pt3.y = point_set_opt[i](1);
    pt3.z = point_set_opt[i](2);
    mk3.points.push_back(pt3);
  }
  pubs_spline[3].publish(mk3);
  ros::Duration(0.001).sleep();

  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    Eigen::Vector3d pt3 = ctrl_pts.row(i).transpose();
    cpt_opt.push_back(pt3);
  }

  visualization_msgs::Marker mk4;
  mk4.header.frame_id  = "world";
  mk4.header.stamp     = ros::Time::now();
  mk4.type             = visualization_msgs::Marker::SPHERE_LIST;
  mk4.action           = visualization_msgs::Marker::DELETE;
  mk4.id               = 400;
  pubs_spline[4].publish(mk4);

  mk4.action             = visualization_msgs::Marker::ADD;
  mk4.pose.orientation.x = 0.0;
  mk4.pose.orientation.y = 0.0;
  mk4.pose.orientation.z = 0.0;
  mk4.pose.orientation.w = 1.0;

  mk4.color.r = 1.0;
  mk4.color.g = 0.6;
  mk4.color.b = 0.15;
  mk4.color.a = 0.8;
  
  mk4.scale.x = 0.2;
  mk4.scale.y = 0.2;
  mk4.scale.z = 0.2;
  
  geometry_msgs::Point pt4;
  for (int i = 0; i < int(cpt_opt.size()); i++) {
    pt4.x = cpt_opt[i](0);
    pt4.y = cpt_opt[i](1);
    pt4.z = cpt_opt[i](2);
    mk4.points.push_back(pt4);
  }
  pubs_spline[4].publish(mk4);
  ros::Duration(0.001).sleep();

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {

    feasible = pos.reallocateTime();

    if (++iter_num >= 3) break;
  }

  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;

  double tn = pos.getTimeSum();

  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();

  // save planned results

  local_data_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  return true;
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.5;
  int    seg_num = ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  const double            forward_t = 5.0;
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; i=i+5) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
