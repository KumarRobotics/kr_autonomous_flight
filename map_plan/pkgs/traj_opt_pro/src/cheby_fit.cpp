#include <traj_opt_pro/cheby_fit.h>
#include <iostream>
namespace traj_opt {

VecD FitCheby::fit_poly(std::vector<decimal_t> &numerical_eval, boost::shared_ptr<BasisTransformer> bt) {
  int n = bt->getN();
  std::vector<decimal_t> pol = getCheby(n);

  MatD A = MatD::Zero(n,n);
  VecD b = VecD::Zero(n,1);



  for(int i=0;i<n;i++) {
    decimal_t point = pol.at(i);
    uint eval_n = std::floor(point*decimal_t(numerical_eval.size()-1));
    b(i) = numerical_eval.at(eval_n);

    for(int j=0;j<n;j++)
      A(i,j) = std::pow(point,j);
  }
  VecD co_raw = A.fullPivLu().solve(b);
  VecD co_real = bt->getBasisBasisTransform()*co_raw;
  std::cout << "fit eqn " << b.transpose() << std::endl;
  std::cout << "co_raw " << co_raw.transpose() << std::endl;


//  return Poly(co_real.data(),n-1);
  return co_real;
}


std::vector<decimal_t> FitCheby::getCheby(int n) {
  std::vector<decimal_t> res;
  for(int k=1;k<=n;k++) {
    decimal_t rat = (2.0*decimal_t(k) - 1.0)/(2.0*decimal_t(n));
    res.push_back(0.5 + 0.5*std::cos(rat*3.1415926535897932384626433832795) );
    std::cout << "cheby " << res.back() << std::endl;
  }
  return res;
}

} // namespace traj_opt
