/**
 * @file math.h
 * @brief Polynomial roots solver

 * Solving real roots for n-th order polynomial:
    if n < 5, the closed form solution will be calculated;
    if n >= 5, using Eigen Polynomials solver which is slower but correct.
 */
#pragma once
#include <mpl_basis/data_type.h>

#include <iostream>
#include <unsupported/Eigen/Polynomials>

namespace MPL {

decimal_t normalize_angle(decimal_t angle);

/// Quadratic equation: \f$b*t^2+c*t+d = 0\f$
std::vector<decimal_t> quad(decimal_t b, decimal_t c, decimal_t d);

/// Cubic equation: \f$a*t^3+b*t^2+c*t+d = 0\f$
std::vector<decimal_t> cubic(decimal_t a, decimal_t b, decimal_t c,
                             decimal_t d);

/// Quartic equation: \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$
std::vector<decimal_t> quartic(decimal_t a, decimal_t b, decimal_t c,
                               decimal_t d, decimal_t e);

/*! \brief General solver for \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$

  \f$a, b, c\f$ can be zero. The function itself checks the highest order of the
  polynomial.
  */
std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c, decimal_t d,
                             decimal_t e);

/// General solver for \f$a*t^5+b*t^4+c*t^3+d*t^2+e*t+f = 0\f$
std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c, decimal_t d,
                             decimal_t e, decimal_t f);

/// General solver for \f$a*t^6+b*t^5+c*t^4+d*t^3+e*t^2+f*t+g = 0\f$
std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c, decimal_t d,
                             decimal_t e, decimal_t f, decimal_t g);

/// Return \f$n!\f$
int factorial(int n);

/// Return \f$t^n\f$
decimal_t power(decimal_t t, int n);

template <typename Derived>
typename Derived::PlainObject pseudoInverse(
    Eigen::MatrixBase<Derived> const &m) {
  // JacobiSVD: thin U and V are only available when your matrix has a dynamic
  // number of columns.
  constexpr auto flags = (Derived::ColsAtCompileTime == Eigen::Dynamic)
                             ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
                             : (Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<typename Derived::PlainObject> m_svd(m, flags);
  // std::cout << "singular values: " << m_svd.singularValues().transpose()
  //           << "\n";
  return m_svd.solve(Derived::PlainObject::Identity(m.rows(), m.rows()));
}

template <typename Derived>
typename Derived::PlainObject matrixSquareRoot(
    Eigen::MatrixBase<Derived> const &mat, bool semidefinite_mat = false) {
  if (!semidefinite_mat) {
    Eigen::LLT<typename Derived::PlainObject> cov_chol{mat};
    if (cov_chol.info() == Eigen::Success) return cov_chol.matrixL();
  }
  Eigen::LDLT<typename Derived::PlainObject> cov_chol{mat};
  if (cov_chol.info() == Eigen::Success) {
    typename Derived::PlainObject const L = cov_chol.matrixL();
    auto const P = cov_chol.transpositionsP();
    auto const D_sqrt = cov_chol.vectorD().array().sqrt().matrix().asDiagonal();
    return P.transpose() * L * D_sqrt;
  }
  return Derived::PlainObject::Zero(mat.rows(), mat.cols());
}

}  // namespace MPL
