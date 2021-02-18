// Copyright 2017 Michael Watterson
#include <traj_opt_pro/nonlinear_solver.h>
// move to cpp
#include <algorithm>
#include <utility>
#include <vector>
#include <nlopt.h>

namespace traj_opt {

// outstreams
std::ostream &operator<<(std::ostream &os, const Variable &var) {
  os << "V" << var.id << " : " << var.val << " ";
  return os;
}
std::ostream &operator<<(std::ostream &os, const EqConstraint &var) {
  for (auto &p : var.coeff) os << *(p.first) << " * " << p.second << " + ";
  os << " = " << var.rhs;
  return os;
}
std::ostream &operator<<(std::ostream &os, const ET &trip) {
  os << trip.row() << " , " << trip.col() << " , " << trip.value();
  return os;
}

std::ostream &operator<<(std::ostream &os, const ETV &trip) {
  for (auto &t : trip) os << t << std::endl;
  return os;
}
// to be able to sort
bool operator==(const Variable &var1, const Variable &var2) {
  return var1.val == var2.val;
}
bool operator<(const Variable &var1, const Variable &var2) {
  return var1.val < var2.val;
}


// Eq solver helpers
ETV EqConstraint::ai() {
  ETV a;
  a.reserve(coeff.size());
  for (auto &p : coeff) a.push_back(ET(var_v->id, p.first->getId(), p.second));
  return a;
}
ET EqConstraint::bi() {
  decimal_t val = -rhs;
  for (auto &p : coeff) val += p.first->val * p.second;
  return ET(var_v->id, 0, val);
}
ETV EqConstraint::audio_video() {
  ETV av;
  for (auto &p : coeff) av.push_back(ET(p.first->id, 0, var_v->val * p.second));
  return av;
}
std::pair<ETV, ET> EqConstraint::get_presolve() {
  ETV a_i;
  ET b_i(id, 0, rhs);
  for (auto &c : coeff) a_i.push_back(ET(id, c.first->id, c.second));
  return std::pair<ETV, ET>(a_i, b_i);
}

// Ineq solver helpers
void IneqConstraint::update_slack() { var_s->val = -1.0 * evaluate(); }

ET IneqConstraint::slack() { return ET(var_u->id, 0, evaluate() + var_s->val); }
ET IneqConstraint::sports_util(decimal_t nu) {
  return ET(var_s->id, 0, var_s->val * var_u->val - nu);
}
ETV IneqConstraint::gradientS() {
  ETV grad = gradient();
  ETV gradS;
  gradS.reserve(grad.size());
  for (auto t : grad) gradS.push_back(ET(t.col(), 0, t.value() * var_u->val));
  return gradS;
}
decimal_t IneqConstraint::linesearch(const VecD &delta, decimal_t max_h) {
  max_h = 1.0;
  decimal_t ui = var_u->val;
  decimal_t dui = delta(var_u->id);
  if (dui > 1e-6) {  // negative dui is fine
    max_h = std::min(ui / dui, max_h);
  }
  decimal_t si = var_s->val;
  decimal_t dsi = delta(var_s->id);
  if (dsi > 1e-6) {  // negative dui is fine
    max_h = std::min(si / dsi, max_h);
  }

  //    if(max_h < 1e-13){
  //        std::cout << "Having trouble with constriant " << id << std::endl;
  //        std::cout << "u: " << ui  << " ,du: " << dui << std::endl;
  //        std::cout << "s: " << si  << " ,ds: " << dsi << std::endl;
  //        // hacky fix
  ////        max_h = 1.0;
  ////        if(dui > 1e-6)
  ////            var_u->val += dui;
  ////        if(dsi > 1e-6)
  ////            var_s->val += dsi;
  //    }

  return max_h;
}

// Nonlinear solver
void NonlinearSolver::addConstraint(boost::shared_ptr<EqConstraint> con) {
  con->id = static_cast<int>(eq_con.size());
  con->var_v = addVar();
  eq_con.push_back(con);
}
void NonlinearSolver::addConstraint(boost::shared_ptr<IneqConstraint> con) {
  con->id = static_cast<int>(ineq_con.size());
  con->var_u = addVar();
  con->var_s = addVar();
  //    con->var_s->val = - con->evaluate();
  ineq_con.push_back(con);
}
void NonlinearSolver::addConstraint(std::vector<EqConstraint::EqPair> con,
                                    decimal_t rhs) {
  boost::shared_ptr<EqConstraint> c = boost::make_shared<EqConstraint>();
  c->coeff = con;
  c->rhs = rhs;
  addConstraint(c);
}

// transposes sparse triple
ETV NonlinearSolver::transpose(const ETV &vec) {
  ETV res;
  res.reserve(vec.size());
  for (auto &v : vec) {
    res.push_back(ET(v.col(), v.row(), v.value()));
  }
  return res;
}
decimal_t NonlinearSolver::duality() {
  decimal_t gap = 0.0;
  int num_u = ineq_con.size();
  for (auto &con : ineq_con) gap += con->var_u->val * con->var_s->val;
  gap /= decimal_t(num_u);
  if(num_u == 0)
    return 0.0;
  else
    return gap;
}

bool NonlinearSolver::iterate(std::vector<Variable *> sensitive_vars) {
  // get variable sizes
  // int num_v = eq_con.size();
  // Timer tm;
  int num_u = ineq_con.size();
  int total_v = vars.size();
  int num_z = total_v - 2*num_u;// - num_v;

  SpMat M(total_v, total_v);
  SpMat b(total_v, 1);
  ETV coeffs;
  ETV bcoeffs;

  VecD nu = VecD::Zero(num_u);
  // add A
  // std::cout << "Frontend pre took " << tm.toc() << std::endl;
  // tm.tic();


  ETV A;
  for (auto &eq : eq_con) {
    ETV ai = eq->ai();
    //        for(auto &aii:ai)
    //            std::cout << aii.row() << " , " << aii.col() << " : " <<
    //            aii.value() << std::endl;
    //        std::cout << "rhs " << eq->rhs << std::endl;
    A.insert(A.end(), ai.begin(), ai.end());
    bcoeffs.push_back(eq->bi());
  }
  ETV AT = transpose(A);
  coeffs.insert(coeffs.end(), A.begin(), A.end());
  coeffs.insert(coeffs.end(), AT.begin(), AT.end());
  
  // std::cout << "Frontend eq took " << tm.toc() << std::endl;
  // tm.tic();

  // add G
  ETV G;
  for (auto &ineq : ineq_con) {
    ETV Gi = ineq->gradient();
    G.insert(G.end(), Gi.begin(), Gi.end());
    bcoeffs.push_back(ineq->slack());
    bcoeffs.push_back(ineq->sports_util(nu(ineq->id)));
    // S,I and Z
    coeffs.push_back(
        ET(ineq->var_s->id, ineq->var_u->id, ineq->var_s->val));  // S
    coeffs.push_back(ET(ineq->var_u->id, ineq->var_s->id, 1.0));  // I
    coeffs.push_back(
        ET(ineq->var_s->id, ineq->var_s->id, ineq->var_u->val));  // Z
  }
  ETV GT = transpose(G);
  if(G.size() > 0) {
    coeffs.insert(coeffs.end(), G.begin(), G.end());
    coeffs.insert(coeffs.end(), GT.begin(), GT.end());
  }

  // std::cout << "Frontend in took " << tm.toc() << " coeffs now " << coeffs.size() <<std::endl;
  // tm.tic();

  // add Cost

  // add to left hand side

  ETV co = cost->hessian();
  coeffs.insert(coeffs.end(), co.begin(), co.end());
  ETV na = cost->gradient();
  bcoeffs.insert(bcoeffs.end(), na.begin(), na.end());
//  std::cout << "cost g size " << bcoeffs.size() << std::endl;
//  std::cout << "cost na size " << na.size() << std::endl;

  // end Add Cost
  // std::cout << "Frontend cost took " << tm.toc() << " coeffs now " << coeffs.size() <<  std::endl;
  // tm.tic();


  // Timer tmm,tmmm;
  // add tensor sum terms
  for (auto &ineq : ineq_con) {
    // tmm.tic();
    ETV coi = ineq->hessian();
    // tmmm.tic();
    ETV gu = ineq->gradientS();

    coeffs.insert(coeffs.end(), coi.begin(), coi.end());
    bcoeffs.insert(bcoeffs.end(), gu.begin(), gu.end());
    // if(tmm.toc() > 0.5) {
    //   std::cout << "inequality took a long time: " << ineq->id << " num values hess "<< coi.size() << " gradient s " << gu.size() << " gradientS time " << tmmm.toc() << std::endl;
    //   ineq->profile();
    // }
  }
  for (auto &eq : eq_con) {
    // ScopedTimer tmm("Inequality av");
    ETV av = eq->audio_video();
    bcoeffs.insert(bcoeffs.end(), av.begin(), av.end());
  }

  // double long_time = tm.toc();
  // std::cout << "Frontend weird things took " << long_time <<" coeffs now " << coeffs.size() << std::endl;
  // tm.tic();
  // if(long_time >2)
  //   throw 2;

  for(int i=0 ; i < bcoeffs.size();i++){
    if(bcoeffs.at(i).row() < 0 || bcoeffs.at(i).row() >= total_v  || bcoeffs.at(i).col() != 0 )
      std::cout << "neg i: " << i << std::endl;
  }

  // pack non summands
  M.setFromTriplets(coeffs.begin(), coeffs.end());
  b.setFromTriplets(bcoeffs.begin(), bcoeffs.end());

  // pass to backend

  // std::cout << "Frontend packing took " << tm.toc() << std::endl;
  // tm.tic();

  // backend bottleneck
  Eigen::SparseLU<SpMat> solver;
//  Eigen::SparseQR<SpMat,Eigen::COLAMDOrdering<int> > solver;
  //    Eigen::PardisoLU<SpMat> solver;

  //        std::cout << "M: " << M << std::endl;
  //        std::cout << "b: " << b << std::endl;
  //    draw_matrix(M);
  solver.compute(M);

  // std::cout << "Back end took " << tm.toc() << std::endl;
  // tm.tic();

  if (solver.info() != Eigen::Success) {
    std::cout << "Back end failed with code " << solver.info() << std::endl;
//            std::cout << M << std::endl;
            draw_matrix(M);
    return false;
  }
  VecD delta_x = solver.solve(b);


  VecD err = M*delta_x;
  err-=b;

  // Check error

  //        std::cout << "delta x aff " << delta_x.transpose() << std::endl;

  // do feasible direction search
  //        MatD x = MatD::Zero(total_v,1);
  //        for(uint i =0 ; i <vars.size();i++)
  //            x(i) = vars.at(i)->val;

  // calculate max distance
  decimal_t max_h = 1.0;

  for (auto &con : ineq_con) {
    decimal_t max_i = con->linesearch(delta_x, max_h);
    max_h = std::min(max_i, max_h);
  }
  for(auto &pv:positive_vars){
    decimal_t max_i = pv->val / delta_x(pv->id);
    if(max_i > 0 )
      max_h = std::min(max_i, max_h);
  }
  //    max_h = std::min(max_h,cost_linesearch(delta_x));
  max_h = std::max(max_h, 0.0);
  //    std::cout << "h aff: " << max_h << std::endl;
  // update state or compute affine

  // calculate nu
  decimal_t mu = duality();
  decimal_t mu_aff = 0.0;
  for (auto &con : ineq_con) {
    decimal_t new_s = con->var_s->val - max_h * delta_x(con->var_s->id);
    decimal_t new_u = con->var_u->val - max_h * delta_x(con->var_u->id);
    if (new_s > 0 && new_u > 0) mu_aff += new_s * new_u;
  }
  mu_aff /= decimal_t(ineq_con.size());
  //    std::cout << "mu: " << mu << std::endl;
  // centering param
  decimal_t sigma = std::pow(mu_aff / mu, 3);
  if(mu == 0.0)
    sigma = 0.0;
  //    std::cout << "Sigma " << sigma << std::endl;

  nu = mu * sigma * VecD::Ones(num_u);
  for (uint i = 0; i < ineq_con.size(); i++)
    nu(i) -=
        delta_x(ineq_con.at(i)->var_u->id) * delta_x(ineq_con.at(i)->var_s->id);

  // calculate state update

  // update b
  for (auto &ineq : ineq_con) {
    ET suv = ineq->sports_util(nu(ineq->id));
    b.coeffRef(suv.row(), suv.col()) = suv.value();
  }
  //     std::cout << "updated b: " << b << std::endl;
  delta_x = solver.solve(b);
  // add watterson hacky hueristic
  for(auto &v:sensitive_vars) {
//     delta_x(v->id) *= std::pow(1.0 - sigma,2.0);
//     if (sigma > 0.5)
//     delta_x(v->id) *= 0.0;
  }


  // redo line search
  max_h = 1.0;
  for (auto &con : ineq_con) {
    decimal_t max_i = con->linesearch(delta_x, max_h);
    max_h = std::min(max_i, max_h);
    if (max_i == 0)
      std::cout << "out of slack for ineq " << con->id << std::endl;
    //         std::cout << "max_i: " << max_i << std::endl;
  }
  for(auto &pv:positive_vars){
    decimal_t max_i = pv->val / delta_x(pv->id);
    std::cout << "pval " << pv->val << " pr " << delta_x(pv->id) << std::endl;
    if(max_i > 0 )
      max_h = std::min(max_i, max_h);
  }

  //     // check duality gap reduction
  //     decimal_t ad=0.0,bd=0.0;

  //     for(auto &con:ineq_con){
  //         ad += delta_x(con->var_u->id)*delta_x(con->var_s->id);
  //         bd += delta_x(con->var_u->id)*con->var_s->val;
  //         bd += delta_x(con->var_s->id)*con->var_u->val;
  //     }
  //     std::cout << "duality diff: h ( h * " <<ad << " - " << bd << " )" <<
  //     std::endl;

  //     max_h = std::min(max_h,cost_linesearch(delta_x));
  max_h = std::max(max_h, 0.0);
  if(mu != 0.0)
    max_h *= 0.99;
//  else
//    std::cout << "delta_x  " << delta_x.transpose() << std::endl;

  //     VecD delta_sub = delta_x.block(0,0,num_z,1);
  //     VecD delta_v = delta_x.block(num_z,0,num_v,1);
  //     VecD delta_us = delta_x.block(num_v+num_z,0,2*num_u,1);

  //      std::cout << "delta_x  " << delta_sub.transpose() << std::endl;
  //      std::cout << "delta_v  " << delta_v.transpose() << std::endl;
  //      std::cout << "delta_us  " << delta_us.transpose() << std::endl;
  //     std::cout << "expected cost diff: " << delta_x.sum()*max_h <<
  //     std::endl;

//  std::cout << "dx: ";
  // update variables
  for (int i = 0; i < total_v; i++) {
    vars.at(i).val -= delta_x(vars.at(i).id) * max_h;
//    if(i < num_z)
//      std::cout << delta_x(vars.at(i).id) * max_h << " ";
  }
//  std::cout << std::endl;
  // robustify against bad search directions
  //     for(uint i =num_z+num_v ; i <total_v;i++){
  //         decimal_t dx = delta_x(vars.at(i).id)*max_h;
  //         if(vars.at(i).val - dx > 0)
  //             vars.at(i).val -= dx;
  //     }

  // central path push
  //     for(auto &con:ineq_con){
  //         con->update_slack();
  //     }
  // check duality gap
  // decimal_t epsilon = 1e-6;
  mu = duality();
  //     std::cout << "max_h: " << max_h << std::endl;
  //     std::cout << "mu: " << mu << std::endl;

  // std::cout << "Frontend 2 took " << tm.toc() << std::endl;
  // tm.tic();

  return (mu > epsilon_) && (max_h > 1e-15);
}
bool NonlinearSolver::solve(bool verbose, decimal_t epsilon, std::vector<Variable*> sensitive_vars) {
  epsilon_ = epsilon;
  int max_iterations = 50;
  // decimal_t nu = 0.5;

  int num_v = eq_con.size();
  int num_u = ineq_con.size();
  int total_v = vars.size();
  int num_z = total_v - 2 * num_u - num_v;
  Timer tm;
  Timer tm_t;
  tm_t.tic();
  if (verbose) {
    std::cout << "Solving model with " << total_v << " variables" << std::endl;
    std::cout << "Primary: " << num_z << std::endl;
    std::cout << "Sensitive: " << sensitive_vars.size() << std::endl;
    std::cout << "Equality: " << num_v << std::endl;
    std::cout << "Inequality: " << num_u << std::endl << std::endl;
    std::cout << "Epsilon: " << epsilon << std::endl;
    std::cout << "Max Iterations: " << max_iterations << std::endl << std::endl;
  }
  //    if(!presolved_)
  //        presolve();
  //        specialized_presolve();
  // decimal_t old_cost = cost->evaluate();
  // VecD old_x = VecD::Zero(num_z);
  // for (int i = 0; i < num_z; i++) old_x(i) = vars.at(i).val;

  // bool costreg=false;
  int its = 0;
  for (int i = 0; i < max_iterations; i++) {
    its = i;
    if (verbose) {
                 std::cout << "Starting iteration " << i << " Gap: " << duality() << std::endl;// << "\033[1A";
      //            std::cout << "x: ";
      //            for(int i=0;i<num_z;i++)
      //                std::cout << vars.at(i).val << " ";
      //            std::cout << std::endl;
      //            std::cout << "v: ";
      //            for(auto &v:eq_con)
      //                std::cout << v->var_v->val << " ";
      //            std::cout << std::endl;
      //            std::cout << "u: ";
      //            for(auto &v:ineq_con)
      //                std::cout << v->var_u->val << " ";
      //            std::cout << std::endl;
      //            std::cout << "s: ";
      //            for(auto &v:ineq_con)
      //                std::cout << v->var_s->val << " ";
      //            std::cout << std::endl;
                  std::cout << "Cost: " << cost->evaluate() << std::endl;
      //            tm.tic();
    }
    if (!iterate(sensitive_vars)) break;

    // check cost regression
    //        decimal_t new_cost = cost->evaluate();

    //        std::cout << "Cost diff: " << old_cost - new_cost << std::endl;
    //        if(old_cost <= new_cost){
    //            for(int i=0;i<num_z;i++)
    //               vars.at(i).val = old_x(i);
    //            costreg = true;
    ////            std::cout << "new cost" << new_cost << std::endl;
    ////            std::cout << "old cost" << old_cost << std::endl;
    //            break;
    //        } else{
    //            for(int i=0;i<num_z;i++)
    //                old_x(i) = vars.at(i).val;
    //            old_cost = new_cost;
    //        }

    //        if(verbose)
    //            std::cout << "Iteration took: " << tm.toc() << std::endl;
  }
  decimal_t mu = duality();
  if (verbose) {
    std::cout << "Solver terminated." << std::endl;
    //        if(costreg)
    //            std::cout << "Due to cost regression" << std::endl;
    //        else
    //            std::cout << "Due to lack of slack" << std::endl;
    std::cout << "x: ";
    for(int i=0;i<num_z;i++)
      std::cout << vars.at(i).val << " ";
    std::cout << std::endl;
    //        std::cout << "v: ";
    //        for(auto &v:eq_con)
    //            std::cout << v->var_v->val << " ";
    //        std::cout << std::endl;
    //        std::cout << "u: ";
    //        for(auto &v:ineq_con)
    //            std::cout << v->var_u->val << " ";
    //        std::cout << std::endl;
    //        std::cout << "s: ";
    //        for(auto &v:ineq_con)
    //            std::cout << v->var_s->val << " ";
    //        std::cout << std::endl;
    std::cout << "Cost: " << cost->evaluate() << std::endl;
    std::cout << "Gap: " << mu << std::endl;
    std::cout << "Iterations: " << its << std::endl;
  }
  if (verbose)
    std::cout << "Total time: " << tm_t.toc() * 1000.0 << " ms." << std::endl;
  return mu <= epsilon_;
}
bool NonlinearSolver::specialized_presolve() {
  // int num_v = eq_con.size();
  // int num_u = ineq_con.size();
  // int total_v = vars.size();
  // int num_z = total_v - 2*num_u - num_v;
  for (auto &con : eq_con) {
    if (con->coeff.size() == 1) {
      con->coeff.front().first->val = con->rhs / con->coeff.front().second;
    } else if (con->coeff.size() == 2) {
      con->coeff.front().first->val =
          (con->rhs - con->coeff.back().first->val * con->coeff.back().second) /
          con->coeff.front().second;
    } else {
      std::cout << "Problem is incompatible with specialized presolve"
                << std::endl;
      return false;
    }
  }

  // add initial slack
  //    for(auto &con:ineq_con) {
  //        con->update_slack();
  //        con->var_u->val = 1.0/con->var_s->val;
  //    }

  //    for(auto&con:eq_con)
  //        std::cout << *con <<std::endl;

  return true;
}

bool NonlinearSolver::presolve() {
  int num_v = eq_con.size();
  int num_u = ineq_con.size();
  int total_v = vars.size();
  int num_z = total_v - 2 * num_u - num_v;
  ETV A, b;
  // add stuff
  for (auto &con : eq_con) {
    std::pair<ETV, ET> pa = con->get_presolve();
    A.insert(A.end(), pa.first.begin(), pa.first.end());
    b.push_back(pa.second);
  }
  SpMat AE(num_v, num_z);
  SpMat bE(num_v, 1);
  AE.setFromTriplets(A.begin(), A.end());
  bE.setFromTriplets(b.begin(), b.end());

  SpMat ATA = AE * AE.transpose();
  SpMat ATb = bE;

  Eigen::SimplicialLLT<SpMat> solver;

  //    std::cout << "M: " << ATA << std::endl;
  //    std::cout << "b: " << ATb << std::endl;

  solver.compute(ATA);
  MatD z_init = AE.transpose() * solver.solve(ATb);
  //    std::cout << "z_init: " << z_init.transpose() << std::endl;

  // add solved variable values
  for (int i = 0; i < num_z; i++) vars.at(i).val = z_init(i);

  // add initial slack
  for (auto &con : ineq_con) con->update_slack();
  for (auto &con : eq_con) std::cout << *con << std::endl;

  return true;
}
decimal_t NonlinearSolver::cost_linesearch(const VecD &delta) {
  std::vector<decimal_t> old_vars;
  for (auto &v : vars) old_vars.push_back(v.val);
  decimal_t old_cost = cost->evaluate();
  decimal_t max_h = 1.0;
  int max_bisects = 50;

  for (int i = 0; i < max_bisects; i++) {
    for (uint j = 0; j < vars.size(); j++) {
      vars.at(j).val = old_vars.at(j) - max_h * delta(j);
    }
    if (cost->evaluate() < old_cost)
      break;
    else
      max_h /= 2.0;
  }
  if (cost->evaluate() >= old_cost) max_h = 0.0;

  // return iterate
  for (uint j = 0; j < vars.size(); j++) {
    vars.at(j).val = old_vars.at(j);
  }
  return max_h;
}
void NonlinearSolver::draw_matrix(const SpMat &mat) {
  int zoom = 5;
  cv::Mat img = cv::Mat(cv::Size(zoom * mat.rows(), zoom * mat.cols()), CV_8U);
  for (int k = 0; k < mat.outerSize(); ++k)
    for (SpMat::InnerIterator it(mat, k); it; ++it)
      for (int zi = 0; zi < zoom; zi++)
        for (int zj = 0; zj < zoom; zj++) {
          if (it.value() > 1e-7)
            img.at<unsigned char>(zoom * it.row() + zi, zoom * it.col() + zj) =
                255;
          else if (it.value() < -1e-7)
            img.at<unsigned char>(zoom * it.row() + zi, zoom * it.col() + zj) =
                128;
          else
            img.at<unsigned char>(zoom * it.row() + zi, zoom * it.col() + zj) =
                0;
        }
  cv::imshow("Matrix", img);
  cv::waitKey(-1);
}
void IneqConstraint::update_max_hess(uint val) {
  max_hess_size = std::max(max_hess_size,val);
}
void IneqConstraint::update_max_grad(uint val){
  max_grad_size = std::max(max_grad_size,val);
}
double NonlinearSolver::nlopt_wrapper_f(uint n, const double *x, double *grad, void *data) {
  NonlinearSolver *solver = static_cast<NonlinearSolver *>(data);
  // set variables
//  if(solver->num_vars() != n)
//    throw std::runtime_error("nlopt num vars doesnt match solver num");
  for(int i=0;i<n;i++){
    solver->vars.at(i).val = x[i];
  }
  double res = solver->cost->evaluate();

  if(grad){
  ETV grade = solver->cost->gradient();

  SpMat gv(n, 1);
  gv.setFromTriplets(grade.begin(),grade.end());
  VecD dy(gv);

    for(int i=0;i<n;i++){
      grad[i] = dy(i);
    }
  }
  return res;
}
double NonlinearSolver::nlopt_wrapper_g(uint n, const double *x, double *grad, void *data) {
  IneqConstraint *ineq = static_cast<IneqConstraint *>(data);
  // set variables

  for(auto &v: ineq->vars) {
    v->val = x[v->id];
  }

  double res = ineq->evaluate();

  if(grad){

  ETV grade = ineq->gradient();

  ETV gradef;
  gradef.reserve(grade.size());

  for(auto &g:grade)
    gradef.push_back(ET(g.col(),0,g.value()));

  SpMat gv(n,1);
  gv.setFromTriplets(gradef.begin(),gradef.end());
  VecD dy(gv);

    for(int i=0;i<n;i++){
      grad[i] = dy(i);
    }
  }



  return res;
}
bool NonlinearSolver::solve_nlopt(){
  Timer tm_t;
  tm_t.tic();

  int n = num_vars() - 2*ineq_con.size();
  nlopt_opt opt;
//  opt = nlopt_create(NLOPT_LD_MMA,n);
  opt = nlopt_create(NLOPT_LN_COBYLA,n);
//  opt = nlopt_create(NLOPT_LN_BOBYQA,n);
  nlopt_set_min_objective(opt,nlopt_wrapper_f,this);

  for(auto &con:ineq_con) {
    nlopt_add_inequality_constraint(opt,nlopt_wrapper_g, con.get(), 1e-3);
  }
  nlopt_set_xtol_rel(opt,1e-4);
//  nlopt_set_maxeval(opt,100);
  nlopt_set_maxtime(opt,5*60);

  std::vector<double> ic(n,1.0);
//  for(int i=0;i<n;i++)
//    ic.at(i) = vars.at(i).getVal();

  double minf;
//  int retc =  -42;
  int retc =  nlopt_optimize(opt,ic.data(),&minf);
  if(retc<0){
    std::cout << "nlopt failed with " << retc << " in " << 1000.0*tm_t.toc() << " ms" << std::endl;
    return false;
  }
  std::cout << "nlopt found min " << minf << " with ret code " << retc <<  " in " << 1000.0*tm_t.toc() << " ms" << std::endl;
  for(int i=0;i<n;i++)
    vars.at(i).val = ic.at(i);

  return true;
}


}  // namespace traj_opt
