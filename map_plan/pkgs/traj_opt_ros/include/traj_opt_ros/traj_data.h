// Copyright 2015 Michael Watterson
#pragma once

#include <string>
#include <vector>

namespace traj_opt {

enum PolyType {
  STANDARD,
  LEGENDRE,
  WATTERSON,
  BEZIER,
  ENDPOINT,
  LIU,
  CHEBYSHEV
};

struct PolynomialData {
  int degree;
  float dt;
  PolyType basis;
  std::vector<float> coeffs;
  // polynomials are stored with parameterization s \in [0,1]
  // time duration dt is used to evaluate polynomial p(t/dt) for t \in [0,dt]

  // define with and without space allocating constructor
  PolynomialData() {}
  explicit PolynomialData(int d) : degree(d), coeffs(degree + 1) {}
};

struct SplineData {
  int segments;
  float t_total;
  // t_total should equal the sum of dt for each segment
  std::vector<PolynomialData> segs;

  // define with and without space allocating constructor
  SplineData() {}
  SplineData(int deg, int seg)
      : segments(seg), segs(segments, PolynomialData(deg)) {}
};

struct TrajData {
  int dimensions;
  std::vector<SplineData> data;
  std::vector<std::string> dimension_names;
  // dimension_names are optional, but are useful for determining what this
  // trajectory parametrizes
  // ex. dimension_names = {'x','y',z'}

  // define with and without space allocating constructor
  TrajData() {}
  TrajData(int dim, int segs, int deg)
      : dimensions(dim), data(dimensions, SplineData(deg, segs)) {}
};

}  // namespace traj_opt
