// Copyright 2015 Michael Watterson
// Implementation File
// Look at polynomial.h for documentation
#include "traj_opt_ros/polynomial_basis.h"

#include <boost/math/special_functions/factorials.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/range/irange.hpp>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace traj_opt {

Poly PolyCalculus::integrate(const Poly &p) {
  // integrates polynomial with 0 as constant of integration
  Poly::size_type rows = p.size();
  std::vector<double> v;
  v.push_back(0.0);
  for (Poly::size_type i = 0; i < rows; i++) {
    double val = static_cast<double>(i + 1);
    v.push_back(p[i] / val);
  }
  Poly result(v.data(), rows);
  return result;
}
Poly PolyCalculus::differentiate(const Poly &p) {
  // differentiates polynomial
  Poly::size_type rows = p.size();
  if (rows <= 1) return Poly(0.0);
  std::vector<double> v;
  for (Poly::size_type i = 1; i < rows; i++) {
    double val = static_cast<double>(i);
    v.push_back(p[i] * val);
  }
  Poly result(v.data(), rows - 2);
  return result;
}

uint Basis::dim() { return n_p; }

// switched these constructors to use new generic one
BasisBundle::BasisBundle(uint n_p_, uint k_r_)
    : BasisBundle(LEGENDRE, n_p_, k_r_) {}
BasisBundle::BasisBundle(int n) : BasisBundle(BEZIER, n, 0) {}

void StandardBasis::differentiate() {
  for (std::vector<Poly>::iterator it = polys.begin(); it != polys.end();
       ++it) {
    *it = PolyCalculus::differentiate(*it);
  }
}
void StandardBasis::integrate() {
  for (std::vector<Poly>::iterator it = polys.begin(); it != polys.end();
       ++it) {
    *it = PolyCalculus::integrate(*it);
  }
}

double StandardBasis::evaluate(double x, uint coeff) const {
  //    std::cout << "poly size " << polys.size() << std::endl;
  assert(coeff < polys.size());

  if (x > 1.0 || x < 0.0)
    throw std::out_of_range(
        "Tried to evaluate shifted legensdre basis out of normalized range "
        "[0,1]");
  return polys[coeff].evaluate(x);
}
std::ostream &operator<<(std::ostream &os, const StandardBasis &lb) {
  for (std::vector<Poly>::const_iterator it = lb.polys.begin();
       it != lb.polys.end(); ++it) {
    os << (*it) << std::endl;
  }
  return os;
}

double BasisBundle::getVal(double x, double dt, uint coeff,
                              int derr) const {
  assert(derr < static_cast<int>(derrivatives.size()));
  if (derr < 0) {
    assert(-derr <= static_cast<int>(integrals.size()));
    if (dt != 0.0) {
      double factor = std::pow(dt, -static_cast<int>(derr));
      return factor * integrals.at(-derr - 1)->evaluate(x, coeff);
    } else {
      return integrals.at(-derr - 1)->evaluate(x, coeff);
    }

  } else {
    if (dt != 0.0) {
      double factor = std::pow(dt, -static_cast<int>(derr));
      return factor * derrivatives.at(derr)->evaluate(x, coeff);
    } else {
      return derrivatives.at(derr)->evaluate(x, coeff);
    }
  }
}
// BasisBundle::~BasisBundle() {}

Basis::Basis(uint n_p_) : n_p(n_p_) { type_ = PolyType::STANDARD; }
// Basis::~Basis() {}

double StandardBasis::innerproduct(uint i, uint j) const {
  Poly L = polys.at(i) * polys.at(j);
  Poly Lint = PolyCalculus::integrate(L);
  return Lint.evaluate(1.0);
}

Poly StandardBasis::getPoly(uint i) const { return polys.at(i); }
StandardBasis::StandardBasis(uint n) : Basis(n) {
  type_ = PolyType::STANDARD;
  if (n == 0) return;
  std::vector<double> simple;
  simple.push_back(1.0);
  for (uint i = 0; i <= n_p; i++) {
    Poly poly(simple.data(), simple.size() - 1);
    polys.push_back(poly);
    simple.back() = 0.0;
    simple.push_back(1.0);
  }
}

boost::shared_ptr<Basis> BasisBundle::getBasis(int i) {
  if (i >= 0)
    return derrivatives.at(i);
  else
    return integrals.at(-i - 1);
}
BasisBundle::BasisBundle(PolyType type, uint n_p_, uint k_r_)
    : n_p(n_p_), k_r(k_r_) {
  // only computer the first 4 derrivatives
  derrivatives.reserve(10);
  // computer the integral too, preferable use i7 to do computering
  integrals.reserve(1);

  for (auto i : boost::irange(0, 11)) {
    boost::shared_ptr<Basis> base;

    if (type == LEGENDRE)
      throw std::runtime_error(
          "Legendre Basis not implemented in basic package");
    else if (type == STANDARD)
      base = boost::make_shared<StandardBasis>(n_p);
    else if (type == BEZIER)
      throw std::runtime_error("Bezier Basis not implemented in basic package");
    else if (type == ENDPOINT)
      throw std::runtime_error(
          "Enpoint Basis not implemented in basic package");
    else if (type == CHEBYSHEV)
      throw std::runtime_error(
          "Chebyshev Basis not implemented in basic package");
    else
      throw std::runtime_error("Unknown basis type");

    if (i == 11) {
      base->integrate();
      integrals.push_back(base);
    } else {
      for (int j = 0; j < i; j++) base->differentiate();
      derrivatives.push_back(base);
    }
  }
}
}  // namespace traj_opt
