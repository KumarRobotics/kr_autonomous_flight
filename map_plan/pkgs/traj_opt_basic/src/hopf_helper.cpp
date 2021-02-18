#include <traj_opt_basic/hopf_helper.h>

namespace traj_opt {

Quat ConvertHopf::getQuat(const Vec3 &xi, decimal_t yaw, bool hover) {
  Vec3 xih = xi;
  if(xih.norm() > 1e-5)
    xih.normalize();
  else
    xih = Vec3::UnitZ();
  double a = xih(0);
  double b = xih(1);
  double c = xih(2);
  Quat orin_yaw = Quat(std::cos(yaw/2.0),0.0,0.0,std::sin(yaw/2.0));
  Quat fib;

  if(hover) {
    double denom = 1.0/std::sqrt(2.0+2.0*c);
    fib =  Quat((1.0+c)*denom,-b*denom,a*denom,0.0);
  } else {
    double denom = 1.0/std::sqrt(2.0-2.0*c);
    fib =  Quat(-b*denom,(1.0-c)*denom,0.0,a*denom);
  }

  // never send nan
  if(std::isnan( fib.w() ) )
    return Quat::Identity();

  return fib*orin_yaw;
}


} // namespace traj_opt
