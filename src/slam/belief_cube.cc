#include "belief_cube.hh"
#include <future>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>
#include "common.hh"
#include "eigen3/Eigen/Dense"
#include "math/math_util.h"
#include "models.hh"
#include "raster_map.hh"
#include "sensor_msgs/LaserScan.h"

namespace {
class ParallelBeliefCubeTask {
 public:
  // TODO: surely this can be refactored
  ParallelBeliefCubeTask(slam::BeliefCube& cube,
                         const slam::RasterMap& ref_map,
                         const Eigen::Vector2f& odom_disp,
                         const double odom_angle_disp,
                         const sensor_msgs::LaserScan& new_obs,
                         const int dtheta_start,
                         const int dtheta_end,
                         const int dx_start,
                         const int dx_end,
                         const int dy_start,
                         const int dy_end)
      : cube(cube),
        ref_map(ref_map),
        odom_disp(odom_disp),
        odom_angle_disp(odom_angle_disp),
        new_obs(new_obs),
        dtheta_start(dtheta_start),
        dtheta_end(dtheta_end),
        dx_start(dx_start),
        dx_end(dx_end),
        dy_start(dy_start),
        dy_end(dy_end) {}

  void target() {
    cube.eval_range(ref_map, odom_disp, odom_angle_disp, new_obs, dtheta_start, dtheta_end,
                    dx_start, dx_end, dy_start, dy_end);
    barrier_.set_value();
  }

  std::function<void()> as_fn() {
    std::function<void()> fn = std::bind(&ParallelBeliefCubeTask::target, this);
    barrier_ = std::promise<void>();
    completed_ = barrier_.get_future();

    return fn;
  }

  void wait() { completed_.wait(); }

 public:
  slam::BeliefCube& cube;
  const slam::RasterMap& ref_map;
  const Eigen::Vector2f& odom_disp;
  const double odom_angle_disp;
  const sensor_msgs::LaserScan& new_obs;
  const int dtheta_start;
  const int dtheta_end;
  const int dx_start;
  const int dx_end;
  const int dy_start;
  const int dy_end;

 private:
  std::promise<void> barrier_;
  std::future<void> completed_;
};
}  // namespace

namespace slam {

decltype(auto) BeliefCube::max_index_iterator() const {
  if (cube_.empty()) {
    throw std::runtime_error("[BeliefCube::max_index_iterator() FATAL]: empty cube");
  }

  auto it = cube_.begin();
  auto max_it = it;
  while (++it != cube_.cend()) {
    if (it->second > max_it->second) {
      max_it = it;
    }
  }
  return max_it;
}

void BeliefCube::eval(const RasterMap& ref_map,
                      const Eigen::Vector2f& odom_disp,
                      const double odom_angle_disp,
                      const sensor_msgs::LaserScan& new_obs) {
  auto __delayedfn = common::runtime_dist().auto_lap("BeliefCube::eval");
  cube_.clear();
  max_belief_.reset();

  eval_range(ref_map, odom_disp, odom_angle_disp, new_obs, -rot_windowsize_, rot_windowsize_ + 1,
             -tx_windowsize_, tx_windowsize_ + 1, -tx_windowsize_, tx_windowsize_ + 1);
}

void BeliefCube::parallel_eval(const RasterMap& ref_map,
                               const Eigen::Vector2f& odom_disp,
                               const double odom_angle_disp,
                               const sensor_msgs::LaserScan& new_obs) {
  auto __delayedfn = common::runtime_dist().auto_lap("BeliefCube::parallel_eval");
  // evaluate four disjoint child cubes
  // rather than merging the cubes, set the max likelihood directly

  // Evaluate quadrants in the x,y space
  const size_t JOBS = 4;
  std::vector<BeliefCube> quadrants;
  std::vector<ParallelBeliefCubeTask> tasks;

  quadrants.reserve(JOBS);
  tasks.reserve(JOBS);

  for (size_t id = 0; id < JOBS; id++) {
    // id is a bitmask that represents [bool is_pos_x, bool is_pos_y]

    const int x_start = (id & 0b10) ? 0 : -tx_windowsize_;
    const int x_end = (id & 0b10) ? tx_windowsize_ + 1 : 0;
    const int y_start = (id & 1) ? 0 : -tx_windowsize_;
    const int y_end = (id & 1) ? tx_windowsize_ + 1 : 0;
    const int theta_start = -rot_windowsize_;
    const int theta_end = rot_windowsize_ + 1;

    quadrants.emplace_back();
    tasks.emplace_back(quadrants.back(), ref_map, odom_disp, odom_angle_disp, new_obs, theta_start,
                       theta_end, x_start, x_end, y_start, y_end);
  }

  for (ParallelBeliefCubeTask& task : tasks) {
    common::thread_pool_exec(task.as_fn());
  }

  // uneven runtimes
  // is there a better way of waiting?
  for (int i = JOBS - 1; i > -1; i--) {
    auto __a =
        common::runtime_dist().auto_lap("BeliefCube::parallel_eval::Join" + std::to_string(i));
    tasks[i].wait();
  }

  // is this dangerous?
  auto max_it = quadrants.front().max_index_iterator();

  for (const BeliefCube& cube : quadrants) {
    auto it = cube.max_index_iterator();
    if (it->second > max_it->second) {
      max_it = it;
    }
  }

  max_belief_ = unbinify(max_it->first);
}

// TODO: refactor
struct Entry {
  Eigen::Vector3i index;
  double prob;

  Entry(const Eigen::Vector3i& i, double p) : index(i), prob(p) {}

  bool operator<(const Entry& other) const { return prob < other.prob; }
};

void BeliefCube::eval_range(const RasterMap& ref_map,
                            const Eigen::Vector2f& odom_disp,
                            const double odom_angle_disp,
                            const sensor_msgs::LaserScan& new_obs,
                            const int dtheta_start,
                            const int dtheta_end,
                            const int dx_start,
                            const int dx_end,
                            const int dy_start,
                            const int dy_end) {
  // not a great threshold, as the standard deviations are dependent
  // on the magnitudes of displacement
  static const double log_motion_prob_threshold = LogNormalPdf(3, 0, 1);

  // approximate the symmetric robust observation likelihood model
  static const double log_obs_prob_threshold = SymmetricRobustLogObsModelThreshold(2.5);

  std::priority_queue<Entry> plausible_entries;

  // Evaluate the motion model first to prune the index space a bit.
  for (int dtheta = dtheta_start; dtheta < dtheta_end; dtheta += rot_resolution_) {
    const int dtheta_index = dtheta / rot_resolution_;

    for (int dx = dx_start; dx < dx_end; dx += tx_resolution_) {
      const int dx_index = dx / tx_resolution_;

      for (int dy = dy_start; dy < dy_end; dy += tx_resolution_) {
        const int dy_index = dy / tx_resolution_;

        const Point index(dx_index, dy_index, dtheta_index);
        const auto [hypothesis_disp, hypothesis_rot] = unbinify(index);

        const double log_motion_prob =
            LogMotionModel(odom_disp, odom_angle_disp, hypothesis_disp, hypothesis_rot);

        if (log_motion_prob > log_motion_prob_threshold) {
          plausible_entries.emplace(index, log_motion_prob);
        }
      }
    }
  }

  // Evaluate the observation likelihood model on plausible indices
  // according to the motion model.
  //
  // Since the only care about the index with the maximum likelihood,
  // we can keep track of the running maximium likelihood and prune
  // while evaluating the model if we know that the current likelihood
  // cannot be greater than the maximum likelihood.
  const std::vector<Eigen::Vector2f> obs_points = PointsFromScan(new_obs);
  double max_prob = -std::numeric_limits<double>::infinity();

  while (!plausible_entries.empty()) {
    Entry e = plausible_entries.top();
    plausible_entries.pop();

    const Point& index = e.index;
    const double log_motion_prob = e.prob;

    const auto [d_loc, d_theta] = unbinify(index);
    const Eigen::Rotation2Df dtheta_rot(d_theta);
    double log_sum = 0;

    // CRITICAL PRUNING ASSUMPTION: ref_map.query (LogObsModel) is never positive
    const double prune_threshold = max_prob - log_motion_prob;
    bool prune = false;

    for (size_t scan_i = 0; scan_i < obs_points.size(); scan_i += 10) {
      const Eigen::Vector2f& point = obs_points[scan_i];

      // Translate the observation into the previous reference frame.
      const Eigen::Vector2f query_point = dtheta_rot * point + d_loc;
      const float query_dist = query_point.norm();
      if (query_dist <= new_obs.range_min || query_dist >= new_obs.range_max) {
        continue;
      }

      double log_obs_prob = ref_map.query(query_point.x(), query_point.y());
      // approximate the symmetric robust observation likelihood model
      log_obs_prob = std::max(log_obs_prob, log_obs_prob_threshold);
      log_sum += log_obs_prob;

      if (log_sum < prune_threshold) {
        prune = true;
        break;
      }
    }

    if (!prune) {
      double belief_prob = log_motion_prob + log_sum;
      max_prob = std::max(max_prob, belief_prob);
      cube_.emplace(index, belief_prob);
    }
  }
}

std::pair<Eigen::Vector2f, double> BeliefCube::max_belief() const {
  auto __delayedfn = common::runtime_dist().auto_lap("BeliefCube::max_belief");
  if (max_belief_.has_value()) {
    return max_belief_.value();
  }

  max_belief_ = unbinify(max_index_iterator()->first);
  return max_belief_.value();
}

int BeliefCube::meters_to_tx_index(const double meters) const {
  // TODO: breaks ties towards positive inf
  // It should probably be either towards or away from 0 instead
  // for positive/negative symmetry.

  const double cm = meters * 100.0;
  const double bin = (cm + tx_resolution_ * 0.5) / tx_resolution_;
  return static_cast<int>(bin);
}

BeliefCube::Point BeliefCube::binify(const double x, const double y, const double rad) const {
  using math_util::RadToDeg;
  using math_util::ReflexToConvexAngle;

  const int x_bin = meters_to_tx_index(x);
  const int y_bin = meters_to_tx_index(y);

  // TOOD: round instead of floor?
  const int theta_bin = static_cast<int>(RadToDeg(ReflexToConvexAngle(rad)));
  return Point(x_bin, y_bin, theta_bin);
}

BeliefCube::Point BeliefCube::binify(const Eigen::Vector2f& coord, const double rad) const {
  return binify(coord.x(), coord.y(), rad);
}

double BeliefCube::tx_index_to_meters(const int tx_index) const {
  const double cm = tx_index * tx_resolution_;
  return cm / 100.0;
}

std::pair<Eigen::Vector2f, double> BeliefCube::unbinify(const Point& index) const {
  const double x_coord = tx_index_to_meters(index.x());
  const double y_coord = tx_index_to_meters(index.y());

  // potential for a bug:
  // we don't account for rot_resolution in binify nor here
  const double rad = math_util::DegToRad(static_cast<double>(index.z()));

  return std::make_pair(Eigen::Vector2f(x_coord, y_coord), rad);
}

}  // namespace slam
