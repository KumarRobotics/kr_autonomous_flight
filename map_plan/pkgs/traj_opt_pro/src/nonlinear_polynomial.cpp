// Copyright 2017 Michael Watterson
#include <traj_opt_pro/nonlinear_trajectory.h>

#include <map>

namespace traj_opt {

SymbolicPoly::SymbolicPoly(PolyArbitrary poly, decimal_t a) {
  std::sort(poly.begin(), poly.end());
  for (int i = 1; i < poly.size(); i++) {
    if (poly.at(i - 1).first == poly.at(i).first) {
      poly.at(i - 1).second += poly.at(i).second;
      poly.erase(poly.begin() + i);
      i--;
    }
  }

  if (std::abs(a) > 1e-11)
    arbitrary_map[poly] = a;
}

SymbolicPoly::SymbolicPoly(Variable *coeff, Variable *time, int n,
                           decimal_t a) {
  PolyTriple p = PolyTriple(coeff, time, n);
  if (std::abs(a) > 1e-11)
    poly_map[p] = a;
}
SymbolicPoly::SymbolicPoly(Variable *coeff0, Variable *coeff1, Variable *time,
                           int n, decimal_t a) {
  PolyQuad p = PolyQuad(coeff0, coeff1, time, n);
  if (std::abs(a) > 1e-11)
    quad_map[p] = a;
}
void SymbolicPoly::add(const SymbolicPoly &rhs) {
  for (auto &p : rhs.poly_map) {
    auto it = poly_map.find(p.first);
    if (it == poly_map.end()) {
      poly_map[p.first] = p.second;
    } else {
      poly_map[p.first] = poly_map[p.first] + p.second;
    }
  }
  for (auto &p : rhs.quad_map) {
    auto it = quad_map.find(p.first);
    // check for oposite order
    PolyQuad back = PolyQuad(std::get<1>(p.first), std::get<0>(p.first),
                             std::get<2>(p.first), std::get<3>(p.first));
    auto it2 = quad_map.find(back);
    if (it == quad_map.end()) {
      quad_map[p.first] = p.second;
    } else if (it2 != quad_map.end()) {
      quad_map[back] = quad_map[back] + p.second;
    } else {
      quad_map[p.first] = quad_map[p.first] + p.second;
    }
  }
  for (auto &p : rhs.arbitrary_map) {
    auto it = arbitrary_map.find(p.first);
    if (it == arbitrary_map.end()) {
      arbitrary_map[p.first] = p.second;
    } else {
      arbitrary_map[p.first] = arbitrary_map[p.first] + p.second;
    }
    // trim excess
    if (std::abs(arbitrary_map[p.first]) < 1e-7)
      arbitrary_map.erase(arbitrary_map.find(p.first));
  }
  // cleanup with constant terms
  coeff += rhs.coeff;
}
SymbolicPoly &SymbolicPoly::operator+=(const SymbolicPoly &rhs) {
  this->add(rhs);
  return *this;
}
bool operator==(const SymbolicPoly &lhs, const SymbolicPoly &rhs) {
  if (lhs.arb_size() != rhs.arb_size())
    return false;
  else if (lhs.coeff != rhs.coeff)
    return false;
  else {
    for (auto &pl : lhs.arbitrary_map) {
      auto it = rhs.arbitrary_map.find(pl.first);
      if (it == rhs.arbitrary_map.end())
        return false;
      else if (std::abs(it->second - pl.second) > 1e-5)
        return false;
      else
        ;
    }
  }
  return true;
}
bool RationalPoly::isNull() {
  if (num_.arb_size() == 0 && den_.arb_size() == 0 &&
      std::abs(num_.coeff) < 1e-8 && std::abs(den_.coeff) < 1e-8)
    return true;
  else
    return false;
}

RationalPoly &RationalPoly::operator+=(const RationalPoly &rhs) {
  if (this->den_ == rhs.den_) {
    this->num_ += rhs.num_;
  } else {
    this->num_ = this->num_ * rhs.den_;
    this->num_ += this->den_ * rhs.num_;
    this->den_ = this->den_ * rhs.den_;
  }
  return *this;
}
SymbolicPoly operator+(const SymbolicPoly &lhs, const SymbolicPoly &rhs) {
  SymbolicPoly p = lhs;
  p += rhs;
  return p;
}
RationalPoly operator+(const RationalPoly &lhs, const RationalPoly &rhs) {
  RationalPoly p = lhs;
  p += rhs;
  return p;
}

SymbolicPoly operator*(decimal_t lhs, const SymbolicPoly &rhs) {
  SymbolicPoly p;
  p.poly_map = rhs.poly_map;
  p.quad_map = rhs.quad_map;
  p.arbitrary_map = rhs.arbitrary_map;
  for (auto &pi : p.poly_map)
    p.poly_map[pi.first] *= lhs;
  for (auto &pi : p.quad_map)
    p.quad_map[pi.first] *= lhs;
  for (auto &pi : p.arbitrary_map)
    p.arbitrary_map[pi.first] *= lhs;
  p.coeff = rhs.coeff * lhs;
  return p;
}
SymbolicPoly operator*(const SymbolicPoly &lhs, const SymbolicPoly &rhs) {
  SymbolicPoly p;
  if (lhs.arb_size() == 0)
    return lhs.coeff * rhs;
  if (rhs.arb_size() == 0)
    return rhs.coeff * lhs;
  for (auto &pl : lhs.arbitrary_map)
    for (auto &pr : rhs.arbitrary_map) {
      SymbolicPoly::PolyArbitrary pa;
      // need to merge polynomials to conserve size
      pa.insert(pa.end(), pl.first.begin(), pl.first.end());
      pa.insert(pa.end(), pr.first.begin(), pr.first.end());

      //      int i=0;
      //      for(auto &pp:pr.first){
      //          while(i < pa.size() && pa.at(i).first->getId() <
      //          pp.first->getId() ) i++; if(i < pa.size() && pa.at(i).first ==
      //          pp.first) {
      //            pa.at(i).second += pp.second;
      //          }
      //          else
      //            pa.push_back(pp);

      //      }
      if (pa.size() == 0) {
        throw 22;
      }

      //      pa.insert(pa.end(),pr.first.begin(),pr.first.end());
      p.add(SymbolicPoly(pa, pl.second * pr.second));
    }
  p.add(lhs.coeff * rhs);
  p.add(rhs.coeff * lhs);
  p.coeff = lhs.coeff * rhs.coeff;

  return p;
}
RationalPoly operator*(decimal_t lhs, const RationalPoly &rhs) {
  RationalPoly p = rhs;
  p.num_ = lhs * p.num_;
  return p;
}
RationalPoly operator*(const RationalPoly &lhs, const RationalPoly &rhs) {
  RationalPoly p;
  if (lhs.den_.arb_size() == 0 && rhs.den_.arb_size() == 0) {
    p.num_ = lhs.num_ * rhs.num_;
  } else if (lhs.den_.arb_size() == 0) {
    p.num_ = lhs.num_ * rhs.num_;
    p.den_ = rhs.den_;
  } else if (rhs.den_.arb_size() == 0) {
    p.num_ = lhs.num_ * rhs.num_;
    p.den_ = lhs.den_;
  } else {
    p.num_ = lhs.num_ * rhs.num_;
    p.den_ = lhs.den_ * rhs.den_;
  }
  return p;
}
RationalPoly operator*(const RationalPoly &lhs, const SymbolicPoly &rhs) {
  RationalPoly p = lhs;
  p.num_ = p.num_ * rhs;
  return p;
}
decimal_t SymbolicPoly::evaluate() {
  decimal_t val = 0.0;
  for (auto &p : poly_map) {
    Variable *coeff = std::get<0>(p.first);
    Variable *time = std::get<1>(p.first);
    int n = std::get<2>(p.first);
    decimal_t a = p.second;
    val += std::pow(time->getVal(), n) * coeff->getVal() * a;
  }
  for (auto &p : quad_map) {
    Variable *coeff0 = std::get<0>(p.first);
    Variable *coeff1 = std::get<1>(p.first);
    Variable *time = std::get<2>(p.first);
    int n = std::get<3>(p.first);
    decimal_t a = p.second;
    val +=
        std::pow(time->getVal(), n) * coeff0->getVal() * coeff1->getVal() * a;
  }
  //        std::cout << "evaluate " << val << std::endl;
  return val;
}
ETV SymbolicPoly::gradient(int u_id) {
  ETV grad;
  std::map<Variable *, decimal_t> time_grad;
  for (auto &p : poly_map) {
    Variable *coeff = std::get<0>(p.first);
    Variable *time = std::get<1>(p.first);
    int n = std::get<2>(p.first);

    decimal_t dt = decimal_t(n) * std::pow(time->getVal(), n - 1) *
                   coeff->getVal() * p.second;
    if (time_grad.find(time) == time_grad.end())
      time_grad[time] = dt;
    else
      time_grad[time] += dt;

    ET gi = ET(u_id, coeff->getId(), p.second * std::pow(time->getVal(), n));
    if (coeff->getId() >= 0)
      grad.push_back(gi);
  }
  for (auto &v : time_grad) {
    ET gt = ET(u_id, v.first->getId(), v.second);
    if (v.first->getId() >= 0)
      grad.push_back(gt);
  }
  return grad;
}
ETV SymbolicPoly::hessian() {
  std::map<Variable *, decimal_t> time_hess;
  ETV hess;
  // note for this form, all second derivatives are zero with respect to the
  // coeffiients
  for (auto &p : poly_map) {
    Variable *coeff = std::get<0>(p.first);
    Variable *time = std::get<1>(p.first);
    int n = std::get<2>(p.first);
    //    if (n > 1) {
    decimal_t dt = decimal_t(n * (n - 1)) * std::pow(time->getVal(), n - 2) *
                   coeff->getVal() * p.second;
    if (time_hess.find(time) == time_hess.end())
      time_hess[time] = dt;
    else
      time_hess[time] += dt;
    //    }
    //    if (n > 0) {
    decimal_t c = decimal_t(n) * std::pow(time->getVal(), n - 1) * p.second;
    if (std::min(coeff->getId(), time->getId()) >= 0) {
      hess.push_back(ET(coeff->getId(), time->getId(), c));
      hess.push_back(ET(time->getId(), coeff->getId(), c));
    }
    //    }
  }
  // d^2g/dtdt
  for (auto &v : time_hess) {
    ET ht = ET(v.first->getId(), v.first->getId(), v.second);
    if (std::min(v.first->getId(), v.first->getId()) >= 0)
      hess.push_back(ht);
  }
  return hess;
}
ETV SymbolicPoly::quad_gradient() {
  return NonlinearSolver::transpose(
      quad_gradient(0)); // for cost function, gradient is single column
}

ETV SymbolicPoly::quad_gradient(int u_id) {
  ETV grad;
  std::map<Variable *, decimal_t> time_grad;
  std::map<Variable *, decimal_t> coeff_grad;

  for (auto &p : quad_map) {
    Variable *coeff0 = std::get<0>(p.first);
    Variable *coeff1 = std::get<1>(p.first);
    Variable *time = std::get<2>(p.first);
    int n = std::get<3>(p.first);
    if (1) {
      decimal_t dt = decimal_t(n) * std::pow(time->getVal(), n - 1) *
                     coeff0->getVal() * coeff1->getVal() * p.second;
      if (time_grad.find(time) == time_grad.end())
        time_grad[time] = dt;
      else
        time_grad[time] += dt;
    }
    if (coeff0->getId() == coeff1->getId()) {
      if (coeff_grad.find(coeff0) == coeff_grad.end())
        coeff_grad[coeff0] =
            2.0 * coeff0->getVal() * p.second * std::pow(time->getVal(), n);
      else
        coeff_grad[coeff0] +=
            2.0 * coeff0->getVal() * p.second * std::pow(time->getVal(), n);

    } else {
      if (coeff_grad.find(coeff0) == coeff_grad.end())
        coeff_grad[coeff0] =
            coeff1->getVal() * p.second * std::pow(time->getVal(), n);
      else
        coeff_grad[coeff0] +=
            coeff1->getVal() * p.second * std::pow(time->getVal(), n);

      if (coeff_grad.find(coeff1) == coeff_grad.end())
        coeff_grad[coeff1] =
            coeff0->getVal() * p.second * std::pow(time->getVal(), n);
      else
        coeff_grad[coeff1] +=
            coeff0->getVal() * p.second * std::pow(time->getVal(), n);
    }
  }
  for (auto &v : coeff_grad) {
    ET gt = ET(u_id, v.first->getId(), v.second);
    if (v.first->getId() >= 0)
      grad.push_back(gt);
  }
  for (auto &v : time_grad) {
    ET gt = ET(u_id, v.first->getId(), v.second);
    if (v.first->getId() >= 0)
      grad.push_back(gt);
  }
  return grad;
}
ETV SymbolicPoly::quad_hessian() {
  ETV hess;
  std::map<Variable *, decimal_t> time_hess;
  for (auto &p : quad_map) {
    Variable *coeff0 = std::get<0>(p.first);
    Variable *coeff1 = std::get<1>(p.first);
    Variable *time = std::get<2>(p.first);
    int n = std::get<3>(p.first);
    // d^2/dt^2
    decimal_t dt = decimal_t(n - 1) * decimal_t(n) *
                   std::pow(time->getVal(), n - 2) * coeff0->getVal() *
                   coeff1->getVal() * p.second;
    if (time_hess.find(time) == time_hess.end())
      time_hess[time] = dt;
    else
      time_hess[time] += dt;
    // d^2/dc/dc
    if (coeff0->getId() == coeff1->getId()) {
      ET gi = ET(coeff0->getId(), coeff0->getId(),
                 2.0 * p.second * std::pow(time->getVal(), n));
      if (coeff0->getId() >= 0)
        hess.push_back(gi);
      // d^2/{dtdc} has 2 terms
      ET gi2 = ET(coeff0->getId(), time->getId(),
                  2.0 * coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi3 = ET(time->getId(), coeff0->getId(),
                  2.0 * coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      if (std::min(coeff0->getId(), time->getId()) >= 0) {
        hess.push_back(gi2);
        hess.push_back(gi3);
      }

    } else {
      ET gi0 = ET(coeff0->getId(), coeff1->getId(),
                  p.second * std::pow(time->getVal(), n));
      ET gi1 = ET(coeff1->getId(), coeff0->getId(),
                  p.second * std::pow(time->getVal(), n));
      if (std::min(coeff0->getId(), coeff1->getId()) >= 0) {
        hess.push_back(gi0);
        hess.push_back(gi1);
      }

      // 4 things  d^2/{dtdc}
      ET gi2 = ET(coeff0->getId(), time->getId(),
                  coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi3 = ET(time->getId(), coeff0->getId(),
                  coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi4 = ET(coeff1->getId(), time->getId(),
                  coeff0->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi5 = ET(time->getId(), coeff1->getId(),
                  coeff0->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      if (std::min(coeff0->getId(), time->getId()) >= 0) {
        hess.push_back(gi2);
        hess.push_back(gi3);
      }
      if (std::min(coeff1->getId(), time->getId()) >= 0) {
        hess.push_back(gi4);
        hess.push_back(gi5);
      }
    }
  }
  // d^2/dt^2
  for (auto &v : time_hess) {
    ET gt = ET(v.first->getId(), v.first->getId(), v.second);
    if (v.first->getId() >= 0)
      hess.push_back(gt);
  }
  return hess;
}
// arbitrary part evaluation
decimal_t SymbolicPoly::arb_evaluate() {
  decimal_t val = coeff;
  //  if(arbitrary_map.size() == 0)
  //    val = coeff;
  for (auto part : arbitrary_map) {
    decimal_t vali = 1.0;
    for (auto mon : part.first) {
      if (mon.second != 0)
        vali *= std::pow(mon.first->getVal(), mon.second);
    }
    val += vali * part.second;
  }
  return val;
}
// arbitrary part gradient, yuck hessian is much worst
ETV SymbolicPoly::arb_gradient(int u_id) {
  ETV gradient;
  if (u_id < 0)
    return gradient;
  for (auto &part : arbitrary_map) {
    // ETV gradi;
    for (auto &mon0 : part.first) {
      decimal_t prod = part.second;
      for (auto &mon1 : part.first) {
        if (mon0.first == mon1.first) {
          if (mon0.second == 0) {
            prod *= 0.0;
          } else {
            if (mon0.second != 1) {
              prod *= static_cast<decimal_t>(mon0.second) *
                      std::pow(mon0.first->getVal(), mon0.second - 1);
            }
          }
        } else {
          if (mon1.second != 0) {
            prod *= std::pow(mon1.first->getVal(), mon1.second);
          }
        }
      }
      if (mon0.first->getId() >= 0)
        gradient.push_back(ET(u_id, mon0.first->getId(), prod));
    }
  }
  return gradient;
}
// note we do alot of checks to prevent sending 0^0 to std::pow
ETV SymbolicPoly::arb_hessian(uint max_hess_size) {
  ETV hessian;
  hessian.reserve(max_hess_size);
  for (auto part : arbitrary_map) {
    for (auto &mon0 : part.first) {
      for (auto &mon1 : part.first) {
        decimal_t prod = part.second;
        for (auto &mon2 : part.first) {
          // 4 cases when taking the hessian of our thing
          if (mon2.first->id == mon0.first->id &&
              mon2.first->id == mon1.first->id) { // one var derrive twice
            if (mon2.second == 0 || mon2.second == 1) {
              prod *= 0.0;
            } else {
              prod *= static_cast<decimal_t>(mon2.second * (mon2.second - 1)) *
                      std::pow(mon2.first->val, mon2.second - 2);
            }

          } else if (mon2.first->id == mon0.first->id ||
                     mon2.first->id ==
                         mon1.first->id) { // current var derrive once,
            // should be be XOR, but elseif order handles this
            if (mon2.second == 0) {
              prod *= 0.0;
            } else {
              prod *= static_cast<decimal_t>(mon2.second) *
                      std::pow(mon2.first->val, mon2.second - 1);
            }
          } else {
            if (mon2.second != 0) { // current var derrive 0 times
              prod *= std::pow(mon2.first->val, mon2.second);
            }
          }
        }
        if (std::min(mon0.first->id, mon1.first->id) >= 0)
          hessian.push_back(ET(mon0.first->id, mon1.first->id, prod));
      }
    }
  }

  return hessian;
}
// computes the hession of poly^2
ETV SymbolicPoly::arb_hessian2() {
  ETV hessian = arb_hessian();
  ETV gradient = arb_gradient(0);
  decimal_t eval = arb_evaluate();
  for (auto &set : hessian)
    set = ET(set.row(), set.col(), 2.0 * eval * set.value());
  for (auto &gi : gradient)
    for (auto &gj : gradient)
      hessian.push_back(ET(gi.col(), gj.col(), 2.0 * gi.value() * gj.value()));
  return hessian;
}
ETV SymbolicPoly::arb_gradient2(int u_id) {
  ETV gradient = arb_gradient(u_id);
  decimal_t eval = arb_evaluate();
  for (auto &g : gradient)
    g = ET(g.row(), g.col(), 2.0 * eval * g.value());
  return gradient;
}

ETV RationalPoly::hessian2() {
  ETV hessian = this->hessian();
  ETV gradient = this->gradient(0);
  decimal_t eval = evaluate();
  for (auto &set : hessian)
    set = ET(set.row(), set.col(), 2.0 * eval * set.value());
  for (auto &gi : gradient)
    for (auto &gj : gradient)
      hessian.push_back(ET(gi.col(), gj.col(), 2.0 * gi.value() * gj.value()));
  return hessian;
}
ETV RationalPoly::gradient2(int u_id) {
  ETV gradient = this->gradient(u_id);
  decimal_t eval = evaluate();
  for (auto &g : gradient)
    g = ET(g.row(), g.col(), 2.0 * eval * g.value());
  return gradient;
}

SymbolicPoly SymbolicPoly::square() {
  SymbolicPoly resultant;
  // do the naive thing, can be speed up by factor of 2 with some intellegence.
  for (auto p1 : poly_map) {
    Variable *coeff1 = std::get<0>(p1.first);
    Variable *time1 = std::get<1>(p1.first);
    decimal_t a1 = p1.second;
    for (auto p2 : poly_map) {
      Variable *coeff2 = std::get<0>(p2.first);
      decimal_t a2 = p2.second;
      // function assumptions
      assert(time1 == std::get<1>(p2.first));
      assert(std::get<2>(p1.first) == 0);
      assert(std::get<2>(p2.first) == 0);
      resultant.add(SymbolicPoly(coeff1, coeff2, time1, 0, a1 * a2));
    }
  }

  for (auto &p1 : arbitrary_map) {
    for (auto &p2 : arbitrary_map) {
      uint i = 0, j = 0;
      PolyArbitrary pa;
      // merge vectors
      while (i < p1.first.size() && j < p2.first.size()) {
        if (p1.first.at(i).first->val < p2.first.at(j).first->val) {
          pa.push_back(p1.first.at(i));
          i++;
        } else if (p1.first.at(i).first->val > p2.first.at(j).first->val) {
          pa.push_back(p2.first.at(j));
          j++;
        } else {
          pa.push_back(
              std::make_pair(p1.first.at(i).first,
                             p1.first.at(i).second + p2.first.at(j).second));
          i++;
          j++;
        }
      }
      while (i < p1.first.size()) {
        pa.push_back(p1.first.at(i));
        i++;
      }
      while (j < p2.first.size()) {
        pa.push_back(p2.first.at(j));
        j++;
      }
      resultant.add(SymbolicPoly(pa, p1.second * p2.second));
    }
  }
  return resultant;
}
SymbolicPoly SymbolicPoly::scale(decimal_t v) {
  SymbolicPoly resultant;
  resultant.poly_map = poly_map;
  resultant.arbitrary_map = arbitrary_map;
  for (auto &p1 : resultant.poly_map) {
    p1.second *= v;
  }
  for (auto &p1 : resultant.arbitrary_map) {
    p1.second *= v;
  }
  return resultant;
}
std::ostream &operator<<(std::ostream &os, const SymbolicPoly &poly) {
  os << "Constant " << poly.coeff << std::endl;
  os << "Linear parts:" << std::endl;
  for (auto &p : poly.poly_map) {
    os << p.second << " * c" << std::get<0>(p.first)->getId() << " t"
       << std::get<1>(p.first)->getId() << " ^ " << std::get<2>(p.first)
       << std::endl;
  }
  os << "Quadratic parts:" << std::endl;
  for (auto &p : poly.quad_map) {
    os << p.second << " * c" << std::get<0>(p.first)->getId() << " * c"
       << std::get<1>(p.first)->getId() << " t" << std::get<2>(p.first)->getId()
       << " ^ " << std::get<3>(p.first) << std::endl;
  }
  os << "Arbitrary parts:" << std::endl;
  for (auto &p : poly.arbitrary_map) {
    os << p.second;
    for (auto &pi : p.first)
      os << " * v" << pi.first->getId() << " ^ " << pi.second;
    os << std::endl;
  }

  return os;
}
std::ostream &operator<<(std::ostream &os, const RationalPoly &poly) {
  os << "Numerator: " << std::endl << poly.num_ << std::endl << std::endl;
  os << "Denominator: " << std::endl << poly.den_ << std::endl << std::endl;
  return os;
}

std::vector<RationalPoly> S2R3Traj::getPhiPhi(const Quat &R2TR1, Variable *v1,
                                              Variable *v2) {
  Mat3 m = R2TR1.inverse().matrix();
  //  SymbolicPoly::PolyArbitrary pc(1.0);// =
  //  SymbolicPoly::PolyArbitrary(1,SymbolicPoly::PolyPair(v1,0) ); // constant
  //  term
  SymbolicPoly pv1 = SymbolicPoly(
      SymbolicPoly::PolyArbitrary(1, SymbolicPoly::PolyPair(v1, 1)), 1.0);
  SymbolicPoly pv2 = SymbolicPoly(
      SymbolicPoly::PolyArbitrary(1, SymbolicPoly::PolyPair(v2, 1)), 1.0);
  SymbolicPoly p2v1 = SymbolicPoly(
      SymbolicPoly::PolyArbitrary(1, SymbolicPoly::PolyPair(v1, 2)), 1.0);
  SymbolicPoly p2v2 = SymbolicPoly(
      SymbolicPoly::PolyArbitrary(1, SymbolicPoly::PolyPair(v2, 2)), 1.0);

  SymbolicPoly den = SymbolicPoly(m(0, 0) + 1) + (-2.0 * m(1, 0)) * pv1 +
                     (-2.0 * m(2, 0)) * pv2 + (1.0 - m(0, 0)) * p2v1 +
                     (1.0 - m(0, 0)) * p2v2;
  SymbolicPoly num1 = SymbolicPoly(-m(0, 1)) + (2.0 * m(1, 1)) * pv1 +
                      (2.0 * m(2, 1)) * pv2 + (m(0, 1)) * p2v1 +
                      (m(0, 1)) * p2v2;
  SymbolicPoly num2 = SymbolicPoly(-m(0, 2)) + (2.0 * m(1, 2)) * pv1 +
                      (2.0 * m(2, 2)) * pv2 + (m(0, 2)) * p2v1 +
                      (m(0, 2)) * p2v2;

  den = den.simplify();
  num1 = num1.simplify();
  num2 = num2.simplify();

  //  std::cout << "num1 e " << num1.arb_evaluate() << std::endl;
  //  std::cout << "num2 e " << num2.arb_evaluate() << std::endl;
  //  std::cout << "den e " << den.arb_evaluate() << std::endl;

  std::vector<RationalPoly> pp;
  pp.push_back(RationalPoly(num1, den));
  pp.push_back(RationalPoly(num2, den));

  return pp;
}
std::vector<RationalPoly> S2R3Traj::getdPhiPhi(const Quat &R2TR1, Variable *v1,
                                               Variable *v2, Variable *dv1,
                                               Variable *dv2) {
  std::vector<RationalPoly> dphiphi;
  std::vector<RationalPoly> phiphi = S2R3Traj::getPhiPhi(R2TR1, v1, v2);
  std::vector<Variable *> vars{v1, v2};
  std::vector<Variable *> dv{dv1, dv2};
  for (int i = 0; i < 2; i++) {
    SymbolicPoly num = phiphi[i].getNum();
    SymbolicPoly den = phiphi[i].getDen();
    RationalPoly res;
    for (int j = 0; j < 2; j++) {
      SymbolicPoly dnum = num.getPartial(vars[j]);
      SymbolicPoly dden = den.getPartial(vars[j]);
      //      std::cout << "d num " << dnum << std::endl;
      //      std::cout << "d den " << dden << std::endl;
      SymbolicPoly rnum = dnum * den + (-1.0) * dden * num;
      SymbolicPoly rden = den * den;
      rnum = rnum.simplify();
      rden = rden.simplify();
      //      std::cout << "dpp part num " << rnum << std::endl;
      //      std::cout << "dpp part den " << rden << std::endl;
      SymbolicPoly dvj = SymbolicPoly(
          SymbolicPoly::PolyArbitrary(1, SymbolicPoly::PolyPair(dv[j], 1)),
          1.0);
      //      std::cout << "dvj " << dvj << std::endl;
      res += RationalPoly(rnum * dvj, rden);
    }
    res = RationalPoly(res.getNum().simplify(), res.getDen().simplify());
    //    std::cout << "dpp num " << res.getNum() << std::endl;
    //    std::cout << "dpp den " << res.getDen() << std::endl;
    dphiphi.push_back(res);
  }
  return dphiphi;
}

RationalPoly::RationalPoly(const SymbolicPoly &num, const SymbolicPoly &den)
    : num_(num), den_(den) {
  for (auto &e : num.arbitrary_map) {
    for (auto &v : e.first) {
      vars_.insert(v.first);
    }
  }
  for (auto &e : den.arbitrary_map) {
    for (auto &v : e.first) {
      vars_.insert(v.first);
    }
  }
}
RationalPoly::RationalPoly(const SymbolicPoly &num) : num_(num) {
  den_.coeff = 1.0;
  for (auto &e : num.arbitrary_map) {
    for (auto &v : e.first) {
      vars_.insert(v.first);
    }
  }
}
void RationalPoly::calcVars() {
  vars_.clear();
  for (auto &part : num_.arbitrary_map) {
    for (auto &pi : part.first) {
      if (pi.first->getId() >= 0)
        vars_.insert(pi.first);
      //      if(pi.first->getId() == 17)
      //        std::cout << std::endl;
    }
  }
  for (auto &part : den_.arbitrary_map) {
    for (auto &pi : part.first) {
      if (pi.first->getId() >= 0)
        vars_.insert(pi.first);
      //      if(pi.first->getId() == 17)
      //        std::cout << std::endl;
    }
  }
}

ETV RationalPoly::gradient(int u_id) {
  //  if(den_.arbitrary_map.size() == 0)
  //    return num_.gradient(u_id);
  calcVars();
  ETV gnum = num_.arb_gradient(u_id);
  ETV gden = den_.arb_gradient(u_id);

  // change to map
  std::map<int, decimal_t> gnum_map, gden_map;
  for (auto &ni : gnum) {
    if (gnum_map.find(ni.col()) == gnum_map.end())
      gnum_map[ni.col()] = ni.value();
    else
      gnum_map[ni.col()] += ni.value();
  }
  for (auto &ni : gden) {
    if (gden_map.find(ni.col()) == gden_map.end())
      gden_map[ni.col()] = ni.value();
    else
      gden_map[ni.col()] += ni.value();
  }
  // do quotient rule
  decimal_t den = den_.arb_evaluate();
  decimal_t num = num_.arb_evaluate();

  ETV resultant;
  for (auto &vi : vars_) {
    if (vi->getId() < 0)
      continue;
    decimal_t vn = 0.0;
    if (gnum_map.find(vi->getId()) != gnum_map.end()) {
      vn += gnum_map[vi->getId()] * den;
    }
    if (gden_map.find(vi->getId()) != gden_map.end()) {
      vn += -gden_map[vi->getId()] * num;
    }
    vn /= den * den;
    resultant.push_back(ET(u_id, vi->getId(), vn));
  }
  return resultant;
}

ETV RationalPoly::hessian() {
  //  if(den_.arbitrary_map.size() == 0)
  //    return num_.hessian();
  calcVars();
  ETV gnum = num_.arb_gradient(0);
  ETV gden = den_.arb_gradient(0);

  // change to map
  std::map<int, decimal_t> gnum_map, gden_map;
  for (auto &ni : gnum) {
    if (gnum_map.find(ni.col()) == gnum_map.end())
      gnum_map[ni.col()] = ni.value();
    else
      gnum_map[ni.col()] += ni.value();
  }
  for (auto &ni : gden) {
    if (gden_map.find(ni.col()) == gden_map.end())
      gden_map[ni.col()] = ni.value();
    else
      gden_map[ni.col()] += ni.value();
  }
  // do quotient rule
  decimal_t den = den_.arb_evaluate();
  decimal_t num = num_.arb_evaluate();

  ETV hnum = num_.arb_hessian();
  ETV hden = den_.arb_hessian();
  // change to map
  std::map<std::pair<int, int>, decimal_t> hnum_map, hden_map;
  for (auto &ni : hnum) {
    std::pair<int, int> key = std::pair<int, int>(ni.row(), ni.col());
    if (hnum_map.find(key) == hnum_map.end())
      hnum_map[key] = ni.value();
    else
      hnum_map[key] += ni.value();
  }
  for (auto &ni : hden) {
    std::pair<int, int> key = std::pair<int, int>(ni.row(), ni.col());
    if (hden_map.find(key) == hden_map.end())
      hden_map[key] = ni.value();
    else
      hden_map[key] += ni.value();
  }
  // calculate hessian
  ETV resultant;
  for (auto &vi : vars_) {
    for (auto &vj : vars_) {
      int i = vi->getId();
      int j = vj->getId();
      std::pair<int, int> in = std::pair<int, int>(i, j);
      // add if statements
      decimal_t gn = 0.0;
      decimal_t dgn = 0.0;
      //      decimal_t gn = gnum_map[i]*den - gden_map[i]*num;
      //      decimal_t dgn = hnum_map[in]*den + gnum_map[i]*gden_map[j] -
      //      hden_map[in]*num - gden_map[i]*gnum_map[j];
      if (gnum_map.find(i) != gnum_map.end()) {
        gn += gnum_map[i] * den;
        if (gden_map.find(j) != gden_map.end()) {
          dgn += gnum_map[i] * gden_map[j];
        }
      }
      if (gden_map.find(i) != gden_map.end()) {
        gn -= gden_map[i] * num;
        if (gnum_map.find(j) != gnum_map.end()) {
          dgn -= gden_map[i] * gnum_map[j];
        }
      }
      if (hnum_map.find(in) != hnum_map.end())
        dgn += hnum_map[in] * den;
      if (hden_map.find(in) != hden_map.end())
        dgn -= hden_map[in] * num;

      // get results
      decimal_t h = dgn / den / den - 2.0 * gn * gden_map[j] / std::pow(den, 3);
      if (i > 0 && j > 0)
        resultant.push_back(ET(i, j, h));
    }
  }
  return resultant;
}
decimal_t RationalPoly::evaluate() {
  //  if(den_.arbitrary_map.size() == 0 && std::abs(den_.coeff)<1e-7)
  //    return num_.arb_evaluate();
  //  else
  decimal_t num = num_.arb_evaluate();
  decimal_t den = den_.arb_evaluate();
  if (den == 0.0)
    return std::numeric_limits<decimal_t>::max();
  return num / den;
}
SymbolicPoly SymbolicPoly::getPartial(Variable *x) {
  SymbolicPoly p;

  for (auto &part : arbitrary_map) {
    // ETV gradi;
    decimal_t coeff = 0.0;
    SymbolicPoly::PolyArbitrary pa;
    for (auto &mon0 : part.first) {
      if (mon0.first->getId() == x->getId()) {
        coeff = part.second * decimal_t(mon0.second);
        if (mon0.second == 0)
          coeff = 0.0;
        else
          pa.push_back(std::make_pair(mon0.first, mon0.second - 1));
      } else {
        pa.push_back(mon0);
      }
    }
    if (coeff != 0.0)
      p.add(SymbolicPoly(pa, coeff));
  }
  return p;
}
SymbolicPoly SymbolicPoly::simplify() {
  SymbolicPoly val;
  val.coeff = coeff;
  for (auto &pi : arbitrary_map) {
    // cleanup polies
    PolyArbitrary pa;
    for (auto &p : pi.first) {
      if (p.second != 0)
        pa.push_back(p);
    }
    if (pa.size() == 0)
      val.coeff += pi.second;
    else if (std::abs(pi.second) > 1e-5) {
      if (val.arbitrary_map.find(pa) != val.arbitrary_map.end()) {
        val.arbitrary_map[pa] += pi.second;
        if (std::abs(val.arbitrary_map[pa]) < 1e-5) {
          val.arbitrary_map.erase(val.arbitrary_map.find(pa));
        }
      } else {
        val.arbitrary_map[pa] = pi.second;
      }
    }
  }
  //  std::cout << "before " << *this << std::endl;
  //  std::cout << "after " << val << std::endl;

  return val;
}
} // namespace traj_opt
