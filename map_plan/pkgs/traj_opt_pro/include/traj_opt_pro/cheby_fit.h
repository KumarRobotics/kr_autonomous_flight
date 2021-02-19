#ifndef CHEBY_FIT_H
#define CHEBY_FIT_H

#include <traj_opt_pro/polynomial_basis.h>

namespace traj_opt {

class FitCheby {
  FitCheby() = delete;

 public:
  // bt should be from chebyshev to endpoint
  static VecD fit_poly(std::vector<decimal_t> &numerical_eval,
                       boost::shared_ptr<BasisTransformer> bt);
  static std::vector<decimal_t> getCheby(int n);  // get cheby nodes
};

}  // namespace traj_opt

#endif  // CHEBY_FIT_H
