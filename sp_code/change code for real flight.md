* kino_replan.launch
```xml
  <!-- global parameters -->
  <arg name="max_vel" default="0.5" />
  <arg name="max_acc" default="0.25" />
```

* kino_algorithm.launch
```xml
  ...
  <!-- kinodynamic path searching -->
    <param name="search/max_tau" value="2" type="double"/>
    <param name="search/init_max_tau" value="3" type="double"/>
  ...
  <!-- trajectory optimization -->
  ...
    <param name="optimization/dist0" value="1.8" type="double"/>
  ...
```

* kinodynamic_astar.cpp
```c++
int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
    ...
    double res = 1 / 2.0, time_res = 1 / 4.0, time_res_init = 1 / 20.0;
    ...
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_ / 4; az <= max_acc_ / 4 + 1e-3; az += max_acc_ * res / 4)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
    ...
}
```

* planner_manager.cpp
```c++
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
    if (dist < 1.5) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}
```

* bspline_optimizer.cpp
```c++
void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}
```
