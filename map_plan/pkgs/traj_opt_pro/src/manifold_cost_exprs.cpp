#include <traj_opt_pro/nested.h>

namespace traj_opt {

NestedSO3::NestedSO3(
    const Quat &R1, const Quat &R2,
    const std::vector<boost::shared_ptr<NestedExpression>> &xi1, int d) {
  f_inputs = xi1;
  dq = R2.inverse() * R1;
  dim = d;
}
decimal_t NestedSO3::f(const std::vector<decimal_t> &x) {
  Vec3 xi;
  xi << x.at(0), x.at(1), x.at(2);
  decimal_t ang = xi.norm();
  if (ang >= 1e-3)
    xi.normalize();
  else
    xi = Vec3::UnitX();
  Eigen::AngleAxis<decimal_t> ae(ang, xi);
  ae = dq * ae;
  Vec3 out = ae.axis() * ae.angle();
  return out(dim);
}

double NestedS2CostPart::f(const std::vector<decimal_t> &val) {
  double c0_0 = val.at(0);
  double c0_1 = val.at(1);
  double c0_2 = val.at(2);
  double c0_3 = val.at(3);
  double c0_4 = val.at(4);
  double c0_5 = val.at(5);
  double c0_6 = val.at(6);
  double c0_7 = val.at(7);
  double c1_0 = val.at(8);
  double c1_1 = val.at(9);
  double c1_2 = val.at(10);
  double c1_3 = val.at(11);
  double c1_4 = val.at(12);
  double c1_5 = val.at(13);
  double c1_6 = val.at(14);
  double c1_7 = val.at(15);

  double t2 = t * t;
  double t3 = t2 * t2;
  double t4 = 1.0 / (dt * dt);
  double t5 = 1.0 / (dt * dt * dt);
  double t6 = 1.0 / (dt * dt * dt * dt);
  double t7 = 1.0 / (dt * dt * dt * dt * dt);
  double t8 = 1.0 / dt;
  double t9 = t * t2 * t3 * t6 * (1.0 / 6.0);
  double t10 = t * t2 * t3 * t7 * 2.0;
  double t11 = 1.0 / (dt * dt * dt * dt * dt * dt);
  double t12 = t * t2 * t3 * t11 * 1.0E1;
  double t13 = t * t3 * t7 * 8.4E1;
  double t14 = 1.0 / (dt * dt * dt * dt * dt * dt * dt);
  double t15 = t * t2 * t3 * t14 * 2.0E1;
  double t17 = t * t2 * (1.0 / 6.0);
  double t18 = t3 * t8 * (2.0 / 3.0);
  double t19 = t * t3 * t4;
  double t20 = t2 * t3 * t5 * (2.0 / 3.0);
  double t21 = t9 + t17 - t18 + t19 - t20;
  double t22 = t2 * (1.0 / 2.0);
  double t23 = t3 * t4 * 5.0;
  double t24 = t * t3 * t5 * 1.0E1;
  double t25 = t2 * t3 * t6 * (1.5E1 / 2.0);
  double t26 = t10 + t22 - t23 + t24 - t25;
  double t27 = t3 * t5 * 2.0E1;
  double t28 = t * t3 * t6 * 4.5E1;
  double t29 = t2 * t3 * t7 * 3.6E1;
  double t30 = t + t12 - t27 + t28 - t29;
  double t31 = t3 * t8 * (1.0 / 6.0);
  double t32 = t * t3 * t4 * (1.0 / 2.0);
  double t33 = t2 * t3 * t5 * (1.0 / 2.0);
  double t34 = t3 * t4 * (5.0 / 2.0);
  double t35 = t * t3 * t5 * 7.0;
  double t36 = t2 * t3 * t6 * (1.3E1 / 2.0);
  double t37 = t10 - t34 + t35 - t36;
  double t38 = t3 * t5 * 1.5E1;
  double t39 = t * t3 * t6 * 3.9E1;
  double t40 = t2 * t3 * t7 * 3.4E1;
  double t41 = t3 * t6 * 3.5E1;
  double t42 = t2 * t3 * t11 * 7.0E1;
  double t43 = t13 + t15 - t41 - t42 + 1.0;
  double t44 = t13 + t15 - t41 - t42;
  double t50 = c0_6 * t21;
  double t51 = c0_4 * t26;
  double t52 = c0_2 * t30;
  double t53 = c0_7 * (t9 - t31 + t32 - t33);
  double t54 = c0_5 * t37;
  double t55 = c0_3 * (t12 - t38 + t39 - t40);
  double t56 = c0_0 * t43;
  double t57 = c0_1 * t44;
  double t16 = t50 + t51 + t52 + t53 - t54 + t55 + t56 - t57;
  double t64 = c1_6 * t21;
  double t65 = c1_4 * t26;
  double t66 = c1_2 * t30;
  double t67 = c1_7 * (t9 - t31 + t32 - t33);
  double t68 = c1_5 * t37;
  double t69 = c1_3 * (t12 - t38 + t39 - t40);
  double t70 = c1_0 * t43;
  double t71 = c1_1 * t44;
  double t45 = t64 + t65 + t66 + t67 - t68 + t69 + t70 - t71;
  double t46 = t * t3 * t6 * 7.0;
  double t47 = t2 * t6 * 4.2E2;
  double t48 = t3 * t11 * 2.1E3;
  double t113 = t * t2 * t7 * 1.68E3;
  double t114 = t * t3 * t14 * 8.4E2;
  double t49 = t47 + t48 - t113 - t114;
  double t58 = t * t2 * t4 * 2.0E1;
  double t59 = t2 * t3 * t6 * (7.0 / 6.0);
  double t60 = t2 * t3 * t7 * 1.4E1;
  double t61 = t3 * t7 * 4.2E2;
  double t62 = t2 * t3 * t14 * 1.4E2;
  double t80 = t * t2 * t6 * 1.4E2;
  double t81 = t * t3 * t11 * 4.2E2;
  double t63 = t61 + t62 - t80 - t81;
  double t85 = t * t2 * t8 * (8.0 / 3.0);
  double t86 = t * t3 * t5 * 4.0;
  double t72 = t22 + t23 + t59 - t85 - t86;
  double t73 = t3 * t5 * 5.0E1;
  double t74 = t - t28 - t58 + t60 + t73;
  double t75 = t * t2 * t4 * 1.0E1;
  double t90 = t3 * t5 * 3.5E1;
  double t76 = t39 - t60 + t75 - t90;
  double t77 = t3 * t6 * 1.95E2;
  double t78 = t3 * t6 * 2.25E2;
  double t93 = t * t2 * t5 * 8.0E1;
  double t94 = t * t3 * t7 * 2.16E2;
  double t79 = t42 + t78 - t93 - t94 + 1.0;
  double t82 = t16 * t16;
  double t83 = t45 * t45;
  double t84 = t82 + t83 + 1.0;
  double t87 = c0_6 * t72;
  double t88 = c0_4 * t74;
  double t96 = t * t3 * t5 * 3.0;
  double t97 = t * t2 * t8 * (2.0 / 3.0);
  double t98 = t34 + t59 - t96 - t97;
  double t89 = c0_7 * t98;
  double t91 = c0_5 * t76;
  double t99 = t * t2 * t5 * 6.0E1;
  double t100 = t * t3 * t7 * 2.04E2;
  double t101 = t42 + t77 - t99 - t100;
  double t92 = c0_3 * t101;
  double t95 = c0_2 * t79;
  double t102 = c0_0 * t63;
  double t122 = c0_1 * t63;
  double t103 = t87 + t88 + t89 + t91 + t92 + t95 + t102 - t122;
  double t104 = 1.0 / (t84 * t84);
  double t116 = t2 * t8 * 8.0;
  double t105 = t - t27 + t46 + t58 - t116;
  double t106 = t2 * t8 * 2.0;
  double t107 = t * t2 * t5 * 1.4E2;
  double t108 = t * t2 * t5 * 2.0E2;
  double t109 = t2 * t5 * 1.8E2;
  double t110 = t3 * t7 * 1.02E3;
  double t111 = t2 * t5 * 2.4E2;
  double t112 = t3 * t7 * 1.08E3;
  double t115 = c0_0 * t49;
  double t117 = t38 - t46 - t75 + t106;
  double t143 = t2 * t4 * 3.0E1;
  double t118 = t13 - t77 + t107 - t143;
  double t145 = t2 * t4 * 6.0E1;
  double t119 = t13 - t78 + t108 - t145 + 1.0;
  double t120 = t * t2 * t6 * 7.8E2;
  double t121 = t * t2 * t6 * 9.0E2;
  double t131 = c0_7 * (t34 + t59 - t96 - t97);
  double t132 = c0_3 * (t42 + t77 - t99 - t100);
  double t133 = c0_0 * (t61 + t62 - t80 - t81);
  double t123 = t87 + t88 + t91 + t95 - t122 + t131 + t132 + t133;
  double t124 = c1_6 * t72;
  double t125 = c1_4 * t74;
  double t126 = c1_5 * t76;
  double t127 = c1_2 * t79;
  double t134 = c1_7 * (t34 + t59 - t96 - t97);
  double t135 = c1_3 * (t42 + t77 - t99 - t100);
  double t136 = c1_0 * (t61 + t62 - t80 - t81);
  double t137 = c1_1 * t63;
  double t128 = t124 + t125 + t126 + t127 + t134 + t135 + t136 - t137;
  double t129 = t81 - t109 - t110 + t120;
  double t130 = t81 - t111 - t112 + t121;
  double t149 = t16 * t123 * 2.0;
  double t150 = t45 * t128 * 2.0;
  double t138 = t149 + t150;
  double t140 = 1.0 / t84;
  double t141 = c1_6 * t105;
  double t142 = c1_7 * t117;
  double t144 = c1_5 * t118;
  double t146 = c1_4 * t119;
  double t147 = c1_0 * t49;
  double t148 = c1_1 * t49;
  double t151 = c0_6 * t105;
  double t152 = c0_7 * t117;
  double t153 = c0_5 * t118;
  double t154 = c0_4 * t119;
  double t155 = c0_1 * t49;
  double t159 = t123 * t123;
  double t160 = t159 * 2.0;
  double t161 = t128 * t128;
  double t162 = t161 * 2.0;
  double t163 = c0_3 * t129;
  double t164 = c0_2 * t130;
  double t165 = -t115 + t151 - t152 - t153 + t154 + t155 + t163 + t164;
  double t166 = t16 * t165 * 2.0;
  double t167 = t138 * t138;
  double t168 = 1.0 / (t84 * t84 * t84);
  double t139 =
      t140 *
          (t115 - c0_1 * t49 - c0_6 * t105 +
           c0_3 * (t109 + t110 - t * t2 * t6 * 7.8E2 - t * t3 * t11 * 4.2E2) +
           c0_2 * (t111 + t112 - t * t2 * t6 * 9.0E2 - t * t3 * t11 * 4.2E2) +
           c0_7 * (t38 - t46 + t106 - t * t2 * t4 * 1.0E1) +
           c0_5 * (t13 + t107 - t2 * t4 * 3.0E1 - t3 * t6 * 1.95E2) -
           c0_4 * (t13 + t108 - t2 * t4 * 6.0E1 - t3 * t6 * 2.25E2 + 1.0)) *
          2.0 +
      t16 * t104 *
          (t160 + t162 + t166 +
           t45 *
               (t141 - t142 - t144 + t146 - t147 + t148 + c1_2 * t130 +
                c1_3 * t129) *
               2.0) *
          2.0 -
      t16 * t167 * t168 * 4.0 +
      t103 * t104 *
          (t16 * t103 * 2.0 + t45 *
                                  (t124 + t125 + t126 + t127 + c1_0 * t63 -
                                   c1_1 * t63 + c1_3 * t101 + c1_7 * t98) *
                                  2.0) *
          4.0;
  double t156 = c1_3 * (t81 - t109 - t110 + t120);
  double t157 = c1_2 * (t81 - t111 - t112 + t121);
  double t158 = t141 - t142 - t144 + t146 - t147 + t148 + t156 + t157;
  double t170 = t45 * t158 * 2.0;
  double t169 = t140 * t158 * 2.0 -
                t45 * t104 * (t160 + t162 + t166 + t170) * 2.0 -
                t104 * t128 * t138 * 4.0 + t45 * t167 * t168 * 4.0;
  double t171 =
      t167 * t168 * 4.0 - t104 *
                              (t160 + t162 + t170 +
                               t16 *
                                   (-t115 + t151 - t152 - t153 + t154 + t155 +
                                    c0_3 * (t81 - t109 - t110 + t120) +
                                    c0_2 * (t81 - t111 - t112 + t121)) *
                                   2.0) *
                              2.0;
  double t0 = t139 * t139 + t169 * t169 + t171 * t171;

  //  std::cout << "sphere part val ";
  //  for(auto &vi:val)
  //    std::cout << vi << " ";
  //  std::cout << std::endl;

  return t0;
}
std::vector<decimal_t> NestedS2CostPart::df(const std::vector<decimal_t> &val) {
  throw std::runtime_error("nested df for s2 not implemented");
}
decimal_t NestedSum::f(const std::vector<decimal_t> &val) {
  if (val.size() != weights.size())
    throw std::runtime_error("weighted sum missmatch");
  decimal_t sum = 0;
  for (uint i = 0; i < val.size(); i++) {
    decimal_t dsum = val.at(i) * weights.at(i);
    sum += dsum;
    if (dsum < 0.0)
      std::cout << "warm sum is less than 0 " << dsum << std::endl;
  }
  return sum;
}
std::vector<decimal_t> NestedSum::df(const std::vector<decimal_t> &val) {
  return weights;
}
} // namespace traj_opt
