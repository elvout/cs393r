#ifndef SRC_UTIL_MATRIX_HASH_HH_
#define SRC_UTIL_MATRIX_HASH_HH_

#include <cinttypes>
#include <cstddef>
#include <functional>
#include <type_traits>

namespace util {

/**
 * A hash function for Eigen matrices, based on the Boost library's
 * hash_combine function.
 *
 * https://www.boost.org/doc/libs/1_77_0/doc/html/hash/reference.html#boost.hash_combine
 * http://boost.sourceforge.net/doc/html/boost/hash_combine.html
 *
 * Although this algorithm is widely used, it's not a _great_ hash
 * function.
 * TODO: use a well-tested external library like xxhash?
 */
template <typename T>
class EigenMatrixHash {
 public:
  /// Compute the hash of the provided matrix.
  size_t operator()(const T& matrix) const {
    using ScalarT = typename T::Scalar;

    size_t hash = 0;

    const std::hash<ScalarT> scalar_hasher;
    const ScalarT* const data_start_p = matrix.data();
    const ScalarT* const data_end_p = data_start_p + matrix.size();

    const ScalarT* elem_p = const_cast<const ScalarT*>(data_start_p);
    for (; elem_p < data_end_p; elem_p++) {
      const size_t elem_hv = scalar_hasher(*elem_p);
      hash ^= elem_hv + magic_number() + (hash << 6) + (hash >> 2);
    }

    return hash;
  }

 private:
  /**
   * Return a size_t with a random distribution of bit values based on
   * the reciprocal of the golden ratio.
   */
  static constexpr size_t magic_number() {
    if (std::is_same<size_t, uint64_t>::value) {
      return 0x9e3779b97f4a7c15;
    } else {
      // assume 32-bit, even though size_t is allowed to be 16-bit
      return 0x9e3779b9;
    }
  }
};

}  // namespace util

#endif  // SRC_UTIL_MATRIX_HASH_HH_
