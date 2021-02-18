


// first need to create Axb class
class AxbConstraint : public IneqConstraint {
  NonlinearTrajectory *traj_;
  int i, j;
  VecD ai_;
  decimal_t bi_;

 public:
  SymbolicPoly poly;
  // i is bezier coeff numer
  // j is trajectory segment
  AxbConstraint(NonlinearTrajectory *traj, int j_, int i_, const VecD &ai,
                decimal_t bi)
      : traj_(traj) {
    j = j_;
    i = i_;
    ai_ = ai;
    bi_ = bi;
    Vec3 g;
    g << 0,0,9.81;
    MatD tf = traj_->basisT->getBasisBasisTransform();
    //        std::cout << "tf "<< tf << std::endl;
    // create polynomial object
    for (int d = 0; d < ai.rows(); d++) {
      for (int k = 0; k <= traj_->deg_; k++) {
        Variable *vk = traj->traj.at(d).at(j).at(k);
        if (vk != NULL && std::abs(ai_(d)) > 1e-4) { // for numerical robustness
          poly.add(SymbolicPoly(vk,traj_->times.at(j), k / 2, tf(i, k) * ai_(d)));
          if(!vk->isConstant())
            vars.push_back(vk);
        } else if(vk == NULL) {
          vk = traj_->lamdas.at(j - !(k%2));
          bi_ += tf(i, k) * ai_(d)*g(d);
          decimal_t xi = traj_->xi_hats.at(j - !(k%2))(d);
          poly.add(SymbolicPoly(vk,traj_->times.at(j), k / 2, xi*tf(i, k) * ai_(d)));
        }
      }
    }
  }

 private:
  decimal_t evaluate() { return poly.evaluate() - bi_; }
  ETV gradient() { return poly.gradient(var_u->getId()); }
  ETV hessian() { return poly.hessian(); }
};
// first need to create Axb class
class BallConstraint : public IneqConstraint {
  NonlinearTrajectory *traj_;
  int i, j;
  decimal_t rhs{0.0};

 public:
  SymbolicPoly poly;
  // i is bezier coeff numer
  // j is trajectory segment
  // constructor for keeping the trajectory inside the ball
  BallConstraint(NonlinearTrajectory *traj, int j_, int i_) : traj_(traj) {
    j = j_;
    i = i_;
    MatD tf = traj_->basisT->getBasisBasisTransform();
    // create polynomial object
    for (int d = 0; d < traj->dim_; d++) {
      SymbolicPoly subpoly;
      for (int k = 0; k <= traj_->deg_; k++) {
        subpoly.add(SymbolicPoly(traj->traj.at(d).at(j).at(k),
                                 traj_->times.at(j), k / 2, tf(i, k)));
      }
      subpoly.add(
          SymbolicPoly(traj->beads.at(d).at(j), traj_->times.at(j), 0, -1.0));
      poly.add(subpoly.square());
    }
    // add radius
    poly.add(SymbolicPoly(traj->beads.at(traj_->dim_).at(j),
                          traj->beads.at(traj->dim_).at(j), traj_->times.at(j),
                          0, -1.0));
  }
  // constructor for keeping the ball outisde the points
  // j is the segment
  BallConstraint(NonlinearTrajectory *traj, int j_, const Vec3 &point,
                 decimal_t robot_r)
      : traj_(traj) {
    j = j_;
    assert(traj->dim_ <= 3);
    for (int d = 0; d < traj->dim_; d++) {
      // just manually expand (x-c)^2
      poly.add(SymbolicPoly(traj->beads.at(d).at(j), traj->beads.at(d).at(j),
                            traj_->times.at(j), 0, -1.0));  // x^2
      poly.add(SymbolicPoly(traj->beads.at(d).at(j), traj_->times.at(j), 0,
                            2.0 * point(d)));  // -2xc
      rhs += -point(d) * point(d);             // +c^2
    }
    // expand -(r+r_robot_r)^2
    poly.add(SymbolicPoly(traj->beads.at(traj_->dim_).at(j),
                          traj->beads.at(traj_->dim_).at(j), traj_->times.at(j),
                          0, 1.0));  // -r^2

    rhs += robot_r * robot_r;  // -robot_r^2
  }

 private:
  decimal_t evaluate() { return poly.evaluate() + rhs; }
  ETV gradient() {
    ETV gradl = poly.gradient(var_u->getId());
    ETV gradq = poly.quad_gradient(var_u->getId());

    gradl.insert(gradl.end(), gradq.begin(), gradq.end());
    return gradl;
  }
  ETV hessian() {
    ETV hessl = poly.hessian();
    ETV hessq = poly.quad_hessian();

    hessl.insert(hessl.end(), hessq.begin(), hessq.end());
    return hessl;
  }
};
class PosTimeConstraint : public IneqConstraint {
  Variable *v_;
  decimal_t eps_;
 public:
  explicit PosTimeConstraint(Variable *v, decimal_t eps = 0.1) : v_(v), eps_(eps) { vars.push_back(v); }
  decimal_t evaluate() { return -1.0 * v_->getVal() + eps_; }
  ETV gradient() {
    ET gt = ET(var_u->getId(), v_->getId(), -1.0);
    return ETV(1, gt);
  }
  ETV hessian() { return ETV(); }
};
class TimeBound : public IneqConstraint {
  decimal_t bound_;

 public:
  TimeBound(const std::vector<Variable *> &v, decimal_t upper_bound)
      : IneqConstraint(), bound_(upper_bound) {
    for (auto &vi : v) vars.push_back(vi);
    //        std::copy(v.begin(),v.end(),vars.begin());
  }
  decimal_t evaluate() {
    decimal_t val = -bound_;
    for (auto &v : vars) val += v->getVal();
    return val;
  }
  ETV gradient() {
    ETV gs;
    for (auto &v : vars) {
      ET gt = ET(var_u->getId(), v->getId(), 1.0);
      gs.push_back(gt);
    }
    return gs;
  }
  ETV hessian() { return ETV(); }
};


// minimum distance from point on trajectory to curve
// this assumes traj_->times are constant
class MinDist : public IneqConstraint {
  NonlinearTrajectory *traj_;
  decimal_t total_time;


 public:
  // need 1 poly object per second, precompute all in constructor
  std::vector<SymbolicPoly> polyies;
  // std::vector<SymbolicPoly> polyies_debug;
  Variable *time_var;
  decimal_t constant;
  VecD point;

  // math is easier if we assume time_var is in [0,times(i)], thus we should store var back and forth

  MinDist(NonlinearTrajectory *traj,Variable *var, const VecD &p, decimal_t distance)
      : traj_(traj) {
    total_time = traj_->getTotalTime();
    decimal_t distance2 = distance*distance;
    time_var = var;
    point = p;

    MatD tf = traj_->basisStandard->getBasisBasisTransform();
           // std::cout << "tf "<< tf << std::endl;
    // create polynomial object

    polyies.resize(traj_->seg_*traj_->dim_); // uncomment this! testing for now with 1 seg 3 dim traj
    // polyies_debug.resize(traj_->dim_);
    constant = - distance2;

    for (int j = 0; j < traj_->seg_; j++) {
      for (int d = 0; d < traj_->dim_; d++) {
        SymbolicPoly poly;
        for (int k = 0; k <= traj_->deg_; k++) {
          for (int i = 0; i <= traj_->deg_; i++) {
              Variable *co = traj->traj.at(d).at(j).at(k);
              Variable *dti = traj_->times.at(j);
              SymbolicPoly::PolyArbitrary pa;
              pa.push_back(std::make_pair(co,1));
              pa.push_back(std::make_pair(time_var,i));
              pa.push_back(std::make_pair(dti,int(k/2-i)));
              poly.add(SymbolicPoly(pa, tf(i, k) ));
            }
          }
          SymbolicPoly::PolyArbitrary pa2;
          pa2.push_back(std::make_pair(traj_->times.at(j),0));
          poly.add(SymbolicPoly(pa2, -p(d)));
          // std::cout << "Poly " << j << " , " << d << " : " << poly << std::endl;
          // polyies.at(j).add(poly.square()); // x^2
          polyies.at(d + traj_->dim_*j).add(poly); //- 2x c  uncomment these, testing with d
          // polyies_debug.at(d).add(poly); // x^2
        }
      }
  }
  void profile() override {
    for(auto &p:polyies) {
      std::cout << "Poly: " << std::endl << p <<std::endl;
    }
  }
 private:
  decimal_t evaluate() { 
    decimal_t time = time_var->val;
    uint i = getTimeSeg();
    // std::cout << "before update " << time << " i: " << i << std::endl;
    update_time(i);
    decimal_t val = constant;
    for(int d=0;d<traj_->dim_;d++)
      val += std::pow(polyies.at(d + traj_->dim_*i).arb_evaluate(),2);
    // VecD test,test2;
    // test2 = VecD::Zero(3);
    // traj_->evaluate(time,0,test);
    // for(int d=0;d<traj_->dim_;d++)
      // test2(d)= polyies.at(d + traj_->dim_*i).arb_evaluate();
    // std::cout << "distance evaluation: " << test2.transpose()  << " time " << time << " seg id " << i << " actuall: " << test.transpose() << " Val: " << val << std::endl;
    // std::cout << "debug evaluation: " << polyies_debug.at(0).arb_evaluate() << " " << polyies_debug.at(1).arb_evaluate() << " " << polyies_debug.at(2).arb_evaluate() << " " << std::endl;
    time_var->val = time;
    return val; 
  }
  ETV gradient() { 
    decimal_t time = time_var->val;
    uint i = getTimeSeg();
    update_time(i);
    ETV gradl;
    for(int d=0;d<traj_->dim_;d++) {
      ETV gradli = polyies.at(d + traj_->dim_*i).arb_gradient2(var_u->getId());
      gradl.insert(gradl.end(), gradli.begin(), gradli.end());
    }
    // ETV gradq = polyies.at(i).quad_gradient(var_u->getId());
    // gradl.insert(gradl.end(), gradq.begin(), gradq.end());
    time_var->val = time;
    return gradl; 
  }
  ETV hessian() { 
    decimal_t time = time_var->val;
    uint i = getTimeSeg();
    update_time(i);
    ETV hessl;
    for(int d=0;d<traj_->dim_;d++) {
      ETV hessli = polyies.at(d + traj_->dim_*i).arb_hessian2();
      hessl.insert(hessl.end(), hessli.begin(), hessli.end());
    }
    // ETV hessq = polyies.at(i).quad_hessian();
    // hessl.insert(hessl.end(), hessq.begin(), hessq.end());
    time_var->val = time;
    update_max_hess(hessl.size());
    return hessl; 
  }
  // gets the segment id of the current time;
  uint getTimeSeg() {
    decimal_t t_val = time_var->getVal();
    for(int res=0;res<traj_->seg_;res++){
      if(t_val < traj_->times.at(res)->getVal())
        return res;
      t_val-=traj_->times.at(res)->getVal();
    }
    return traj_->seg_ - 1;
  }
  void update_time(uint i){
    if(i==0)
      return;
    for(uint res=1;res<=i;res++)
      time_var->val -= traj_->times.at(res-1)->getVal();
  }
};
