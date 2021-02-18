// Copyright 2017 Michael Watterson
#include <traj_opt_basic/msg_traj.h>
#include <traj_opt_pro/nonlinear_solver.h>
#include <traj_opt_pro/polynomial_basis.h>
#include <traj_opt_pro/trajectory_solver.h>

#include <boost/algorithm/clamp.hpp>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#ifndef TRAJ_OPT_PRO_NONLINEAR_TRAJECTORY_H_
#define TRAJ_OPT_PRO_NONLINEAR_TRAJECTORY_H_

namespace traj_opt {

typedef std::vector<Variable *> NLTimes;
typedef std::vector<Variable *> NLPoly;
typedef std::vector<NLPoly> NLSpline;
typedef std::vector<NLSpline> NLTraj;

// to maintain same tensor ordering of (dim,segment) for ball confinement
typedef std::vector<Variable *> NLChain;  // (xc,yc,zc,radius)
typedef std::vector<NLChain> NLBeads;
// to constrain point distance, lets add more vars
typedef std::vector<Variable *> NLEval;  // (t_0, t_1, ....)
// for hopf fibration yaw planning
typedef std::vector<VecD> PolySpline;
typedef std::vector<PolySpline> PolyTraj;

class AxbConstraint;
class BallConstraint;
class MinDist;
class RationalPoly;

class SymbolicPoly {
 public:
  typedef std::pair<Variable *, int>
      PolyPair;  // used as a building block for arbitrary monomials
  typedef std::tuple<Variable *, Variable *, int>
      PolyTriple;  // for linear polys in c
  typedef std::tuple<Variable *, Variable *, Variable *, int>
      PolyQuad;  // for quadratic polys in c

  typedef std::vector<PolyPair> PolyArbitrary;

  SymbolicPoly simplify();
  SymbolicPoly(decimal_t coeff_ = 0.0) { coeff = coeff_; }  // null constructor
  // form of a*sum(coeff*t^n)
  SymbolicPoly(PolyArbitrary poly, decimal_t a);
  // form of a*coeff*t^n
  SymbolicPoly(Variable *coeff, Variable *time, int n, decimal_t a);
  // form of a*coeff0*coeff1*t^n
  SymbolicPoly(Variable *coeff0, Variable *coeff1, Variable *time, int n,
               decimal_t a);
  SymbolicPoly &operator+=(const SymbolicPoly &rhs);
  friend SymbolicPoly operator*(decimal_t lhs, const SymbolicPoly &rhs);
  friend SymbolicPoly operator*(const SymbolicPoly &lhs,
                                const SymbolicPoly &rhs);
  friend bool operator==(const SymbolicPoly &lhs, const SymbolicPoly &rhs);
  decimal_t evaluate();
  // for these derrivaties assume only linear terms
  ETV gradient(int u_id);
  ETV hessian();

  // for these derrivaties assume only quadratic terms
  ETV quad_gradient(int u_id);
  ETV quad_gradient();
  ETV quad_hessian();

  decimal_t arb_evaluate();
  ETV arb_gradient(int u_id);
  ETV arb_hessian(uint max_hess_size = 0);

  ETV arb_gradient2(int u_id);
  ETV arb_hessian2();

  SymbolicPoly getPartial(Variable *x);  // returns partial with respect to x

  SymbolicPoly square();            // returns the square of the linear parts
  SymbolicPoly scale(decimal_t v);  // returns v*poly
  void add(const SymbolicPoly &rhs);
  friend std::ostream &operator<<(std::ostream &os, const SymbolicPoly &poly);
  friend class RationalPoly;
  uint arb_size() const { return arbitrary_map.size(); }

 private:
  std::map<PolyTriple, decimal_t> poly_map;
  std::map<PolyQuad, decimal_t> quad_map;
  std::map<PolyArbitrary, decimal_t> arbitrary_map;
  decimal_t coeff = 0.0;
};
class RationalPoly {
 public:
  RationalPoly(const SymbolicPoly &num, const SymbolicPoly &den);
  explicit RationalPoly(const SymbolicPoly &num);
  RationalPoly() { den_.coeff = 1.0; }
  RationalPoly &operator+=(const RationalPoly &rhs);
  decimal_t evaluate();
  // for these derrivaties assume only linear terms
  ETV gradient(int u_id);
  ETV hessian();
  ETV gradient2(int u_id);  // gradient of rational^2
  ETV hessian2();           // hessian of rational^2
  friend std::ostream &operator<<(std::ostream &os, const RationalPoly &poly);
  SymbolicPoly getNum() { return num_; }
  SymbolicPoly getDen() { return den_; }
  friend RationalPoly operator*(decimal_t lhs, const RationalPoly &rhs);
  friend RationalPoly operator*(const RationalPoly &lhs,
                                const RationalPoly &rhs);
  friend RationalPoly operator+(const RationalPoly &lhs,
                                const RationalPoly &rhs);
  friend RationalPoly operator*(const RationalPoly &lhs,
                                const SymbolicPoly &rhs);

 protected:
  SymbolicPoly num_, den_;
  std::set<Variable *> vars_;
  bool isNull();
  void calcVars();
};
typedef std::vector<RationalPoly>
    RPoly;  // these objects still represent coefficients, they are just
            // complicated now
typedef std::vector<NLPoly> RSpline;
typedef std::vector<NLSpline> RTraj;

class PolyCost;
class TimeBound;
class NonlinearTrajectory : public Trajectory {
 public:
  // standard contruction
  NonlinearTrajectory()
      : basis(PolyType::ENDPOINT, 7, 3) {}  // only used with inheritance
  NonlinearTrajectory(
      const std::vector<Waypoint> &waypoints,
      const std::vector<std::pair<MatD, VecD>> &cons, int deg = 7,
      int min_dim = 3,
      boost::shared_ptr<std::vector<decimal_t>> ds =
          boost::shared_ptr<std::vector<decimal_t>>(),
      boost::shared_ptr<VecDVec> path = boost::shared_ptr<VecDVec>());
  // nonconvex pointcloud test
  NonlinearTrajectory(const std::vector<Waypoint> &waypoints,
                      const Vec3Vec &points, int segs, decimal_t dt);
  // map visibility
  NonlinearTrajectory(const std::vector<Waypoint> &waypoints,
                      const std::vector<std::pair<MatD, VecD>> &con,
                      const Vec3Vec &points, decimal_t dt);
  // hopf stuff
  //   NonlinearTrajectory(const std::vector<WaypointHopf> &waypoints,
  //                       const std::vector<std::pair<MatD, VecD>> &con,
  //                       decimal_t dt);

  virtual ~NonlinearTrajectory() {}
  decimal_t getTotalTime() const;
  virtual bool evaluate(decimal_t t, uint derr, VecD &out) const;
  decimal_t getCost();
  virtual TrajData serialize() override;
  bool isSolved() { return solved_; }
  Vec4Vec getBeads();
  void scaleTime(decimal_t ratio);
  void adjustTimeToMax(decimal_t max_v, decimal_t max_a, decimal_t max_j);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  //   TrajData serialize_hopf(const boost::shared_ptr<Trajectory> &traj);
  std::vector<Waypoint> waypoints_;
  void find_charts();
  void allocate_poly(
      boost::shared_ptr<std::vector<decimal_t>> ds =
          boost::shared_ptr<std::vector<decimal_t>>(),
      boost::shared_ptr<VecDVec> path = boost::shared_ptr<VecDVec>(),
      bool marginalize = false, bool use_knots = false, bool use_all = false,
      bool convex = true);
  void allocate_beads();
  void allocate_yaw(const std::vector<decimal_t> &yaws);
  void allocate_evaluations(uint num_eval);
  void link_sections();  // note, requires endpoint basis
  //   void allocate_poly_hopf(const std::vector<decimal_t> &ds);
  void add_boundary(const std::vector<Waypoint> &waypoints,
                    bool marginalize = false);  // add boundary values
  void add_Axb(const std::vector<std::pair<MatD, VecD>> &cons);
  void addVisibility(const Vec3Vec &points, decimal_t dist);
  void addCloudConstraint(const Vec3Vec &points);
  void addPosTime();   // ensures time segments are > 0
  void addPosLamda();  // ensures lamda are > 0
  void addTimeBound(boost::shared_ptr<std::vector<decimal_t>> ds =
                        boost::shared_ptr<std::vector<decimal_t>>());
  void make_convex(boost::shared_ptr<std::vector<decimal_t>> ds);
  void merge_trajectories();
  //   std::pair<Vec3, Vec3> getHopf(decimal_t t);

  //   PolyTraj fitHopfYaw();
  NonlinearSolver solver{10000};
  NonlinearSolver solver_yaw{1000};
  NLTraj trajs2;
  NLTraj traj, traj_yaw;
  NLTimes times, times_yaw;
  NLBeads beads;
  NLEval evaluation_points;
  NLTimes lamdas;
  VecDVec xi_hats;
  std::vector<decimal_t> yaw_dts, yaws;
  std::vector<bool> yaw_charts;
  std::vector<decimal_t> yaw_jumps, yawd_jumps;  // jumps from change of chart

  boost::shared_ptr<PolyCost> cost, cost_yaw;
  boost::shared_ptr<BasisTransformer> basisT;
  boost::shared_ptr<BasisTransformer> basisStandard;  // standard basis tf
  // sizes
  // dim_ in parent
  int seg_, deg_, yaw_seg_;
  BasisBundlePro basis;
  bool solved_{false};
  friend class AxbConstraint;
  friend class BallConstraint;
  friend class MinDist;
};

class SphereCost;
class S2R3Traj : public NonlinearTrajectory {
 public:
  //  S2R3Traj(const std::vector<std::pair<MatD, VecD> > &cons);
  S2R3Traj(const std::vector<std::pair<MatD, VecD>> &cons, Vec5 testn,
           Vec5 testf,
           const std::vector<Quat, Eigen::aligned_allocator<Quat>> &chartss,
           const std::vector<decimal_t> &ds);
  //  virtual bool evaluate(decimal_t t, uint derr, VecD &out) const override;
  //  // add exponential map to this
  TrajData serialize() override;
  static std::vector<RationalPoly> getPhiPhi(const Quat &R2TR1, Variable *v1,
                                             Variable *v2);  // specific to S2
  static std::vector<RationalPoly> getdPhiPhi(const Quat &R2TR1, Variable *v1,
                                              Variable *v2, Variable *dv1,
                                              Variable *dv2);  // specific to S2
  // for s2, can ignore ddphiphi
 protected:
  void allocate_poly(const Vec5 &pn, const Vec5 &pf,
                     const std::vector<decimal_t> &ds);
  void add_Axb(const std::vector<std::pair<MatD, VecD>> &cons, bool r3 = true);
  boost::shared_ptr<SphereCost> scost;
  boost::shared_ptr<PolyCost> r3cost;

  NonlinearSolver solvers2{10000};

  std::vector<Quat, Eigen::aligned_allocator<Quat>> charts;
};

class PolyCost : public CostFunction {
 public:
  PolyCost(const NLTraj &traj, const NLTimes &times, BasisBundlePro &basis,
           int min_dim, decimal_t alpha = 0.0, int max_dim = -1);
  PolyCost(const NLTraj &traj, const NLTimes &lamdas, const NLTimes &times,
           BasisBundlePro &basis, const VecDVec &xi_hats);
  PolyCost(const NLTraj &traj, const NLTimes &times, BasisBundlePro &basis,
           const PolyTraj &fit_yaw);
  decimal_t evaluate();
  ETV gradient();
  ETV hessian();

  int min_dim{3};

 private:
  SymbolicPoly poly;
  MatD cost_v_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cost_n_;
  void init_constants();
  decimal_t constant{0.0};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace traj_opt

#endif  // TRAJ_OPT_PRO_NONLINEAR_TRAJECTORY_H_
