#ifndef SRC_RD_SLAM_LSS_HH_
#define SRC_RD_SLAM_LSS_HH_

#include "math/line2d.h"

namespace rd_slam {

/**
 * Line Segment Similarity Metric
 *
 * Measures the similarity between two lines using distance metrics in subspaces
 * of SE(2).
 *
 * The `strict` flag is used to enforce hard constraints on the parallel and
 * perpendicular distance metrics. It should generally be set to true when
 * establishing correspondences and false when running optimization.
 *
 * Since the range of |sin| is [0, 1], the rotation distance metric is scaled by
 * the ratio of line segment lengths to discourage correspondences between
 * segments with very dissimilar lengths.
 */
double LSS(geometry::line2f a, geometry::line2f b, bool strict = true);

}  // namespace rd_slam

#endif  // SRC_RD_SLAM_LSS_HH_
