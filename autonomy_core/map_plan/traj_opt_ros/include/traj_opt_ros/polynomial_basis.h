// Copyright 2015 Michael Watterson
#pragma once

#include <boost/make_shared.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <boost/math/tools/polynomial.hpp>
#include <cassert>
#include <cmath>
#include <memory>

#include "traj_opt_ros/traj_data.h"
#include "traj_opt_ros/types.h"

namespace traj_opt {

// Basis virtual class
// //////////////////////////////////////////////////////////////////////////
class Basis {
 public:
  explicit Basis(uint n_p_);  // dimension of basis
  virtual void differentiate() = 0;
  virtual void integrate() = 0;

  // evaluates ith element of basis at x where x is normalized to 0 to 1
  virtual double evaluate(double x, uint i) const = 0;
  // innerproduct bewtween ith and jth basis functions
  virtual double innerproduct(uint i, uint j) const = 0;

  virtual uint dim();
  const PolyType &type() const { return type_; }
  bool orthogonal() { return orthogonal_; }

 protected:
  uint n_p;
  PolyType type_;
  bool orthogonal_{false};
};

// Poly Calculus Functions
// //////////////////////////////////////////////////////////////////////////

typedef boost::math::tools::polynomial<double> Poly;

class PolyCalculus {
 public:
  static Poly integrate(const Poly &p);
  // indefinate integral of polynomial with constant 0
  static Poly differentiate(const Poly &p);
  // differentiates polynomial
};

class BasisBundle {  // bundles the basis with its derrivatives
 public:
  // constructs using arbitrary basis
  BasisBundle(PolyType type, uint n_p_, uint k_r_);
  // constructs using legendre
  BasisBundle(uint n_p_, uint k_r_);
  // constructs using bezier
  explicit BasisBundle(int n);
  BasisBundle() {}

  //  ~BasisBundle();
  double getVal(double x, double dt, uint coeff,
                int derr) const;  // returns value of basis at value x, with
                                  // time dt, basis function coeff, and
                                  // derrivative derr
  boost::shared_ptr<Basis> getBasis(int i);

  std::vector<boost::shared_ptr<Basis>> derrivatives;
  std::vector<boost::shared_ptr<Basis>> integrals;

 protected:
  uint n_p, k_r;

  //  std::vector<LegendreBasis> derrivatives;
};

// Polynomial Bases
// A trig basis could be implemented, but we can approximate sin and cos with
// a Lagrange error bound of 1e-6 with a 7th order polynomial over the input
// domain 0 to pi/4.
// //////////////////////////////////////////////////////////////////////////

// 1, t, t^2, ...
class StandardBasis : public Basis {
 public:
  explicit StandardBasis(uint n_p_);
  virtual void differentiate();
  virtual void integrate();
  virtual double evaluate(double x, uint coeff) const;
  friend std::ostream &operator<<(std::ostream &os, const StandardBasis &lb);
  virtual double innerproduct(uint i, uint j) const;
  virtual Poly getPoly(uint i) const;

 protected:
  std::vector<Poly> polys;  // basis polynomials
  uint k_r;
};

}  // namespace traj_opt
