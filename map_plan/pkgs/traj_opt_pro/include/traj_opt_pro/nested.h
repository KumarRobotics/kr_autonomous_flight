#ifndef TRAJ_OPT_PRO_NESTED_H_
#define TRAJ_OPT_PRO_NESTED_H_

#include <traj_opt_pro/nonlinear_solver.h>
#include <traj_opt_pro/nonlinear_trajectory.h>

namespace traj_opt {

class NestedExpression {
 protected:
  Variable *var_;
  decimal_t constant_{0.0};
  // TODO implement d^2 f
  virtual std::vector<decimal_t> df(const std::vector<decimal_t> &val) {
    return std::vector<decimal_t>();
  }
  virtual decimal_t f(const std::vector<decimal_t> &val) {
    return var_->getVal();
  }  // derrivative
  std::vector<boost::shared_ptr<NestedExpression> > f_inputs;
  NestedExpression() {}

 public:
  NestedExpression(Variable *var) : var_(var) {}
  NestedExpression(decimal_t constant) : constant_(constant) {}

  virtual decimal_t evaluate() {
    if (f_inputs.size() == 0) {
      if (var_ != NULL) return var_->getVal();
      return constant_;
    } else {
      std::vector<decimal_t> in;
      in.reserve(f_inputs.size());
      for (auto &p : f_inputs) in.push_back(p->evaluate());
      return f(in);
    }
  }
  virtual ETV gradient() {
    if (f_inputs.size() == 0) {
      if (var_ != NULL) {
        return ETV(1, ET(var_->getId(), 0, 1.0));
      } else {
        return ETV();
      }
    } else {
      std::vector<decimal_t> in;
      in.reserve(f_inputs.size());
      for (auto &p : f_inputs) in.push_back(p->evaluate());
      std::vector<decimal_t> grad = df(in);
      ETV res;
      for (int i = 0; i < f_inputs.size(); i++) {
        ETV val = f_inputs.at(i)->gradient();
        for (auto &p : val)
          res.push_back(ET(p.row(), p.col(), p.value() * grad.at(i)));
      }
      return res;
    }
  }
};
// Helper class for sphere stuff
class RationalNested : public NestedExpression {
 private:
  RationalPoly poly_;

 public:
  RationalNested(RationalPoly poly) : poly_(poly) {}
  ETV gradient() override { return poly_.gradient(0); }
  decimal_t evaluate() override { return poly_.evaluate(); }
};

class NestedS2CostPart : public NestedExpression {
 private:
  decimal_t dt, t;

 public:
  NestedS2CostPart(const std::vector<boost::shared_ptr<NestedExpression> > &in,
                   decimal_t te, decimal_t dte)
      : t(te), dt(dte) {
    f_inputs = in;
  }

 protected:
  decimal_t f(const std::vector<decimal_t> &val) override;
  std::vector<decimal_t> df(const std::vector<decimal_t> &val) override;
};
class NestedSum : public NestedExpression {
 private:
  std::vector<decimal_t> weights;

 public:
  NestedSum(const std::vector<boost::shared_ptr<NestedExpression> > &in,
            const std::vector<decimal_t> &weightse) {
    f_inputs = in;
    weights = weightse;
  }

 protected:
  decimal_t f(const std::vector<decimal_t> &val) override;
  std::vector<decimal_t> df(const std::vector<decimal_t> &val) override;
};
class NestedSO3 : public NestedExpression {
 private:
  // R_1 exp(xi_1) = R_2 exp(xi_2)
  // (do this part in matlab)
  Quat dq;
  int dim;

 public:
  NestedSO3(const Quat &R1, const Quat &R2,
            const std::vector<boost::shared_ptr<NestedExpression> > &xi1,
            int d);

 protected:
  decimal_t f(const std::vector<decimal_t> &x) override;
};

class NestedAxbConstraint : public IneqConstraint {
  int i_, j_;
  VecD ai_;
  decimal_t bi_, dt_;
  std::vector<boost::shared_ptr<NestedExpression> > cx_, cy_;
  boost::shared_ptr<BasisTransformer> basisT;

 public:
  NestedAxbConstraint(std::vector<boost::shared_ptr<NestedExpression> > cx,
                      std::vector<boost::shared_ptr<NestedExpression> > cy,
                      int j, int i, const VecD &ai, decimal_t bi,
                      decimal_t dt) {
    j_ = j;
    i_ = i;
    ai_ = ai;
    bi_ = bi;
    cx_ = cx;
    cy_ = cy;
    dt_ = dt;
    auto endpoint_basis = boost::make_shared<EndPointBasis>(7);
    auto bezier_basis = boost::make_shared<BezierBasis>(7);
    basisT = boost::make_shared<BasisTransformer>(endpoint_basis, bezier_basis);
  }
  decimal_t evaluate() override {
    VecD xe = VecD::Zero(cx_.size(), 1);
    VecD ye = VecD::Zero(cy_.size(), 1);
    for (int c = 0; c < cx_.size(); c++) {
      xe(c) = cx_.at(c)->evaluate() * std::pow(dt_, c / 2);
      ye(c) = cy_.at(c)->evaluate() * std::pow(dt_, c / 2);
    }
    MatD tf = basisT->getBasisBasisTransform();
    VecD tfi = tf.block(i_, 0, 1, cx_.size()).transpose();
    decimal_t xv = xe.dot(tfi);
    decimal_t yv = ye.dot(tfi);
    return ai_(4) * xv + ai_(5) * yv - bi_;
  }
  ETV gradient() override {
    throw std::runtime_error("Gradient not supported.");
    return ETV();
  }
  ETV hessian() override {
    throw std::runtime_error("Hessian not supported.");
    return ETV();
  }
};

}  // namespace traj_opt

#endif  // TRAJ_OPT_PRO_NESTED_H_
