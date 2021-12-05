#ifndef SRC_RD_SLAM_IEPF_HH_
#define SRC_RD_SLAM_IEPF_HH_

#include <vector>
#include "eigen3/Eigen/Dense"
#include "shared/math/line2d.h"

namespace rd_slam {

/**
 * Extract line segments from a 2D observation point cloud.
 *
 * Lines are detected by iterating forward through the columns of the point
 * cloud matrix. Thus, the matrix should be sorted in some fashion. The point
 * cloud generation routine in `models::Observations` sorts the points by
 * counterclockwise scan angle.
 *
 * Returns a vector containing extracted lines in the same order as the point
 * cloud matrix. The endpoints of the lines are also ordered.
 *
 * The iterative end-point fit algorithm implicitly takes four hyperparameters:
 *  - The outlier point distance threshold.
 *  - The minimum number of points per line.
 *  - The minimum line segment length.
 *  - The maximum distance between successive points within line segment.
 *
 * These hyperparameters are defined as constants in the corresponding
 * implementation file. However, these hyperparameters are dependent on the
 * environment, so parameterizing this function accordingly is probably a good
 * idea.
 */
std::vector<geometry::line2f> iterative_end_point_fit(
    const Eigen::Matrix<float, 2, Eigen::Dynamic>& point_cloud);

}  // namespace rd_slam

#endif  // SRC_RD_SLAM_IEPF_HH_
