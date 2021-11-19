#ifndef SRC_MODELS_NORMDIST_HH_
#define SRC_MODELS_NORMDIST_HH_

/**
 * Normal Distribution functions.
 * Most functions are optimized with precomputed values.
 */

namespace models {

double NormalPdf(const double val, const double mean, const double stddev);
double NormalCdf(const double val, const double mean, const double stddev);

/**
 * Natural log of the Normal PDF function, not the Log-Normal PDF function.
 *
 * Computes the log-likelihood directly, allowing for greater precision than
 * log(NormalPdf(...)).
 */
double LnOfNormalPdf(const double val, const double mean, const double stddev);

}  // namespace models

#endif  // SRC_MODELS_NORMDIST_HH_
