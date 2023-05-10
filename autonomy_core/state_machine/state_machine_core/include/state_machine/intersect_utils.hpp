#pragma once
// include data_type, need to use Vec3f
#include <kr_planning_rviz_plugins/data_type.h>

// modified from http://www.3dkingdoms.com/weekly/weekly.php?a=3
namespace state_machine {

inline bool CheckPointInBox(Vec3f B1, Vec3f B2, Vec3f P1) {
  if (P1[0] > B1[0] && P1[0] < B2[0] && P1[1] > B1[1] && P1[1] < B2[1] &&
      P1[2] > B1[2] && P1[2] < B2[2]) {
    return true;
  } else {
    return false;
  }
}

inline int GetIntersection(
    float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f* HitPtr) {
  if ((fDst1 * fDst2) >= 0.0f) return 0;
  if (fDst1 == fDst2) return 0;
  *HitPtr = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));
  return 1;
}

inline int InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis) {
  if (Axis == 1 && Hit[2] > B1[2] && Hit[2] < B2[2] && Hit[1] > B1[1] &&
      Hit[1] < B2[1])
    return 1;
  if (Axis == 2 && Hit[2] > B1[2] && Hit[2] < B2[2] && Hit[0] > B1[0] &&
      Hit[0] < B2[0])
    return 1;
  if (Axis == 3 && Hit[0] > B1[0] && Hit[0] < B2[0] && Hit[1] > B1[1] &&
      Hit[1] < B2[1])
    return 1;
  return 0;
}

// returns true if line (L1, L2) intersects with the box (B1, B2)
// returns intersection point in Hit
inline bool IntersectLineBox(
    Vec3f B1, Vec3f B2, Vec3f L1, Vec3f L2, Vec3f* HitPtr) {
  if (L2[0] < B1[0] && L1[0] < B1[0]) return false;
  if (L2[0] > B2[0] && L1[0] > B2[0]) return false;
  if (L2[1] < B1[1] && L1[1] < B1[1]) return false;
  if (L2[1] > B2[1] && L1[1] > B2[1]) return false;
  if (L2[2] < B1[2] && L1[2] < B1[2]) return false;
  if (L2[2] > B2[2] && L1[2] > B2[2]) return false;

  if ((GetIntersection(L1[0] - B1[0], L2[0] - B1[0], L1, L2, HitPtr) &&
       InBox(*HitPtr, B1, B2, 1)) ||
      (GetIntersection(L1[1] - B1[1], L2[1] - B1[1], L1, L2, HitPtr) &&
       InBox(*HitPtr, B1, B2, 2)) ||
      (GetIntersection(L1[2] - B1[2], L2[2] - B1[2], L1, L2, HitPtr) &&
       InBox(*HitPtr, B1, B2, 3)) ||
      (GetIntersection(L1[0] - B2[0], L2[0] - B2[0], L1, L2, HitPtr) &&
       InBox(*HitPtr, B1, B2, 1)) ||
      (GetIntersection(L1[1] - B2[1], L2[1] - B2[1], L1, L2, HitPtr) &&
       InBox(*HitPtr, B1, B2, 2)) ||
      (GetIntersection(L1[2] - B2[2], L2[2] - B2[2], L1, L2, HitPtr) &&
       InBox(*HitPtr, B1, B2, 3)))
    return true;

  return false;
}
}  // namespace state_machine
