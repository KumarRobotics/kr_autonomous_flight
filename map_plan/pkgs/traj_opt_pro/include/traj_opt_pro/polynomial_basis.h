// Copyright 2015 Michael Watterson
#ifndef TRAJ_OPT_PRO_POLYNOMIAL_BASIS_H_
#define TRAJ_OPT_PRO_POLYNOMIAL_BASIS_H_

#include <traj_opt_basic/polynomial_basis.h>

#include <boost/make_shared.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <boost/math/tools/polynomial.hpp>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

namespace traj_opt {

class PolyCalculusPro : public PolyCalculus {
 public:
  static Poly bernstein_polynomial(typename Poly::size_type n,
                                   typename Poly::size_type i);
  // see wikipedia page
  static Poly chebyshev_polynomial(typename Poly::size_type);
  // see wikipedia page
  static Poly shifted_legendre(typename Poly::size_type n);
  // shifted legensdre polynomial of order n
};

// shifted Legendre bais
class LegendreBasis : public StandardBasis {
 public:
  LegendreBasis(uint n_p_, uint k_r_);
  decimal_t innerproduct(uint i, uint j) const override;
};
// bezier basis
class BezierBasis : public StandardBasis {
 public:
  explicit BezierBasis(uint n_p_);
};
// basis specified by endpoint containers
class EndPointBasis : public StandardBasis {
 public:
  explicit EndPointBasis(uint n_p_);
};
// Chebyshevs
class ChebyshevBasis : public StandardBasis {
 public:
  explicit ChebyshevBasis(uint n_p_);
};

class BasisBundlePro : public BasisBundle {
 public:
  BasisBundlePro(PolyType type, uint n_p_, uint k_r_);
};

class BasisTransformer {
 public:
  BasisTransformer(boost::shared_ptr<StandardBasis> from,
                   int derr = 0);  // default to standard basis
  BasisTransformer(boost::shared_ptr<StandardBasis> from,
                   boost::shared_ptr<StandardBasis> to, int derr = 0);
  const MatD &getBasisBasisTransform();
  const MatD &getLinearTransform(
      decimal_t a,
      decimal_t b);  // transforms coefficents by p(at+b) assuming a \neq 0
  int getN() { return n; }

 private:
  MatD basisbasis_, scaledtranform_;
  MatD A, B, Ainv, Binv;
  boost::shared_ptr<StandardBasis> to_, from_;
  int n;
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_POLYNOMIAL_BASIS_H_
