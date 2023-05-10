#include <primitive_ros_utils.h>

#include <vector>

kr_planning_msgs::Primitive toPrimitiveROSMsg(const MPL::Primitive2D& pr,
                                              double z) {
  const auto cx = pr.pr(0).coeff();
  const auto cy = pr.pr(1).coeff();
  const auto cyaw = pr.pr_yaw().coeff();
  Vec6f cz = Vec6f::Zero();
  cz[5] = z;
  kr_planning_msgs::Primitive msg;
  msg.cx.resize(6);
  msg.cy.resize(6);
  msg.cz.resize(6);
  msg.cyaw.resize(6);
  for (int i = 0; i < 6; i++) {
    msg.cx[i] = cx(i);
    msg.cy[i] = cy(i);
    msg.cz[i] = cz(i);
    msg.cyaw[i] = cyaw(i);
  }
  msg.t = pr.t();

  return msg;
}

kr_planning_msgs::Primitive toPrimitiveROSMsg(const MPL::Primitive3D& pr) {
  const auto cx = pr.pr(0).coeff();
  const auto cy = pr.pr(1).coeff();
  const auto cz = pr.pr(2).coeff();
  const auto cyaw = pr.pr_yaw().coeff();
  kr_planning_msgs::Primitive msg;
  msg.cx.resize(6);
  msg.cy.resize(6);
  msg.cz.resize(6);
  msg.cyaw.resize(6);
  for (int i = 0; i < 6; i++) {
    msg.cx[i] = cx(i);
    msg.cy[i] = cy(i);
    msg.cz[i] = cz(i);
    msg.cyaw[i] = cyaw(i);
  }
  msg.t = pr.t();

  return msg;
}

kr_planning_msgs::PrimitiveArray toPrimitiveArrayROSMsg(
    const vec_E<MPL::Primitive2D>& prs, double z) {
  kr_planning_msgs::PrimitiveArray msg;
  for (const auto& pr : prs) {
    msg.primitives.push_back(toPrimitiveROSMsg(pr, z));
  }
  return msg;
}

kr_planning_msgs::PrimitiveArray toPrimitiveArrayROSMsg(
    const vec_E<MPL::Primitive3D>& prs) {
  kr_planning_msgs::PrimitiveArray msg;
  for (const auto& pr : prs) {
    msg.primitives.push_back(toPrimitiveROSMsg(pr));
  }
  return msg;
}

kr_planning_msgs::Trajectory toTrajectoryROSMsg(const MPL::Trajectory2D& traj,
                                                double z) {
  kr_planning_msgs::Trajectory msg;
  for (const auto& seg : traj.segs)
    msg.primitives.push_back(toPrimitiveROSMsg(seg, z));

  if (traj.lambda().exist()) {
    auto l = traj.lambda();
    msg.lambda.resize(l.segs.size());
    for (int i = 0; i < static_cast<int>(l.segs.size()); i++) {
      msg.lambda[i].dT = l.segs[i].dT;
      msg.lambda[i].ti = l.segs[i].ti;
      msg.lambda[i].tf = l.segs[i].tf;
      msg.lambda[i].ca.resize(4);
      for (int j = 0; j < 4; j++) msg.lambda[i].ca[j] = l.segs[i].a(j);
    }
  }
  return msg;
}

kr_planning_msgs::Trajectory toTrajectoryROSMsg(const MPL::Trajectory3D& traj) {
  kr_planning_msgs::Trajectory msg;
  for (const auto& seg : traj.segs)
    msg.primitives.push_back(toPrimitiveROSMsg(seg));

  if (traj.lambda().exist()) {
    auto l = traj.lambda();
    msg.lambda.resize(l.segs.size());
    for (int i = 0; i < static_cast<int>(l.segs.size()); i++) {
      msg.lambda[i].dT = l.segs[i].dT;
      msg.lambda[i].ti = l.segs[i].ti;
      msg.lambda[i].tf = l.segs[i].tf;
      msg.lambda[i].ca.resize(4);
      for (int j = 0; j < 4; j++) msg.lambda[i].ca[j] = l.segs[i].a(j);
    }
  }
  return msg;
}

MPL::Primitive2D toPrimitive2D(const kr_planning_msgs::Primitive& pr) {
  Vec6f cx, cy, cyaw;
  for (int i = 0; i < 6; i++) {
    cx(i) = pr.cx[i];
    cy(i) = pr.cy[i];
    cyaw(i) = pr.cyaw[i];
  }
  vec_E<Vec6f> cs;
  cs.push_back(cx);
  cs.push_back(cy);
  cs.push_back(cyaw);

  return MPL::Primitive2D(cs, pr.t, MPL::SNPxYAW);
}

MPL::Primitive3D toPrimitive3D(const kr_planning_msgs::Primitive& pr) {
  Vec6f cx, cy, cz, cyaw;
  for (int i = 0; i < 6; i++) {
    cx(i) = pr.cx[i];
    cy(i) = pr.cy[i];
    cz(i) = pr.cz[i];
    cyaw(i) = pr.cyaw[i];
  }
  vec_E<Vec6f> cs;
  cs.push_back(cx);
  cs.push_back(cy);
  cs.push_back(cz);
  cs.push_back(cyaw);

  return MPL::Primitive3D(cs, pr.t, MPL::SNPxYAW);
}

MPL::Trajectory2D toTrajectory2D(const kr_planning_msgs::Trajectory& traj_msg) {
  // Constructor from ros msg
  MPL::Trajectory2D traj;
  traj.taus.push_back(0);
  for (const auto& it : traj_msg.primitives) {
    traj.segs.push_back(toPrimitive2D(it));
    traj.taus.push_back(traj.taus.back() + it.t);
  }

  if (!traj_msg.lambda.empty()) {
    MPL::Lambda l;
    for (int i = 0; i < static_cast<int>(traj_msg.lambda.size()); i++) {
      MPL::LambdaSeg seg;
      seg.a(0) = traj_msg.lambda[i].ca[0];
      seg.a(1) = traj_msg.lambda[i].ca[1];
      seg.a(2) = traj_msg.lambda[i].ca[2];
      seg.a(3) = traj_msg.lambda[i].ca[3];
      seg.ti = traj_msg.lambda[i].ti;
      seg.tf = traj_msg.lambda[i].tf;
      seg.dT = traj_msg.lambda[i].dT;
      l.segs.push_back(seg);
      traj.total_t_ += seg.dT;
    }
    traj.lambda_ = l;
    std::vector<double> ts;
    for (const auto& tau : traj.taus) ts.push_back(traj.lambda_.getT(tau));
    traj.Ts = ts;
  } else {
    traj.total_t_ = traj.taus.back();
  }
  return traj;
}

MPL::Trajectory3D toTrajectory3D(const kr_planning_msgs::Trajectory& traj_msg) {
  MPL::Trajectory3D traj;
  traj.taus.push_back(0);
  for (const auto& it : traj_msg.primitives) {
    traj.segs.push_back(toPrimitive3D(it));
    traj.taus.push_back(traj.taus.back() + it.t);
  }

  if (!traj_msg.lambda.empty()) {
    MPL::Lambda l;
    for (int i = 0; i < static_cast<int>(traj_msg.lambda.size()); i++) {
      MPL::LambdaSeg seg;
      seg.a(0) = traj_msg.lambda[i].ca[0];
      seg.a(1) = traj_msg.lambda[i].ca[1];
      seg.a(2) = traj_msg.lambda[i].ca[2];
      seg.a(3) = traj_msg.lambda[i].ca[3];
      seg.ti = traj_msg.lambda[i].ti;
      seg.tf = traj_msg.lambda[i].tf;
      seg.dT = traj_msg.lambda[i].dT;
      l.segs.push_back(seg);
      traj.total_t_ += seg.dT;
    }
    traj.lambda_ = l;
    std::vector<double> ts;
    for (const auto& tau : traj.taus) ts.push_back(traj.lambda_.getT(tau));
    traj.Ts = ts;
  } else {
    traj.total_t_ = traj.taus.back();
  }
  return traj;
}
