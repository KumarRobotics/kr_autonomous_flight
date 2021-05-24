#include <bitset>
#include <iostream>

namespace MPL {
enum Control {
  VEL = 0b00001,
  ACC = 0b00011,
  JRK = 0b00111,
  SNP = 0b01111,
  VELxYAW = 0b10001,
  ACCxYAW = 0b10011,
  JRKxYAW = 0b10111,
  SNPxYAW = 0b11111
};
}

typedef union {
  struct {
    bool use_pos : 1;
    bool use_vel : 1;
    bool use_acc : 1;
    bool use_jrk : 1;
    bool use_yaw : 1;
  };

  MPL::Control use_xxx : 5;

} USE;

void print(USE u) {
  std::cout << "use_pos | use_vel | use_acc | use_jrk | use_yaw : "
            << std::endl;
  std::cout << u.use_pos << " | " << u.use_vel << " | " << u.use_acc << " | "
            << u.use_jrk << " | " << u.use_yaw << std::endl;
  std::bitset<5> x(u.use_xxx);
  std::cout << "raw use_xxx: " << u.use_xxx << std::endl;
  std::cout << "use_xxx: " << x << std::endl;
  if (u.use_xxx == MPL::VEL)
    std::cout << "use vel!" << std::endl;
  else if (u.use_xxx == MPL::ACC)
    std::cout << "use acc!" << std::endl;
  else if (u.use_xxx == MPL::JRK)
    std::cout << "use jrk!" << std::endl;
  else if (u.use_xxx == MPL::SNP)
    std::cout << "use snp!" << std::endl;
  else if (u.use_xxx == MPL::VELxYAW)
    std::cout << "use vel & yaw!" << std::endl;
  else if (u.use_xxx == MPL::ACCxYAW)
    std::cout << "use acc & yaw!" << std::endl;
  else if (u.use_xxx == MPL::JRKxYAW)
    std::cout << "use jrk & yaw!" << std::endl;
  else if (u.use_xxx == MPL::SNPxYAW)
    std::cout << "use snp & yaw!" << std::endl;
  else
    std::cout << "use null!" << std::endl;
  // std::cout << "size of Control: " << sizeof(u.use_xxx) << std::endl;
}

int main() {
  USE u1, u2, u3, u4, u5, u6;

  print(u1);

  u2.use_pos = true;
  u2.use_vel = false;
  u2.use_acc = false;
  u2.use_jrk = false;

  print(u2);

  u3.use_pos = true;
  u3.use_vel = true;
  u3.use_acc = false;
  u3.use_jrk = false;

  print(u3);

  u4.use_pos = true;
  u4.use_vel = true;
  u4.use_acc = true;
  u4.use_jrk = false;

  print(u4);

  u5.use_pos = true;
  u5.use_vel = true;
  u5.use_acc = true;
  u5.use_jrk = true;

  print(u5);

  u6.use_pos = true;
  u6.use_vel = true;
  u6.use_acc = true;
  u6.use_jrk = true;
  u6.use_yaw = true;

  print(u6);

  std::cout << "VEL_CONTROL: " << MPL::VEL << std::endl;
  std::cout << "ACC_CONTROL: " << MPL::ACC << std::endl;
  std::cout << "JRK_CONTROL: " << MPL::JRK << std::endl;
  std::cout << "SNP_CONTROL: " << MPL::SNP << std::endl;
  return 0;
}
