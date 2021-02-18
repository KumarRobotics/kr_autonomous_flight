#include <stdio.h>

typedef union {
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
} RGBValue;

typedef unsigned char uchar;

class rgb3 {
 public:
  uchar r;  // from 0 ~ 255
  uchar g;  // from 0 ~ 255
  uchar b;  // from 0 ~ 255
  char a;   // from -1 ~ 127

  void print() { printf("r = %d, g = %d, b = %d, a = %d\n", r, g, b, a); }

  rgb3(uchar x0 = 127, uchar x1 = 127, uchar x2 = 127, char x3 = 63)
      : r(x0), g(x1), b(x2), a(x3) {}

  rgb3(float rgb) {
    RGBValue rgb_value;
    rgb_value.float_value = rgb;
    r = rgb_value.Red;
    g = rgb_value.Green;
    b = rgb_value.Blue;
    a = rgb_value.Alpha * 127.;
  }

  rgb3 operator+(const rgb3 &c) {
    if (a <= 0)
      return c;
    else {
      r += 0.9 * (c.r - r);
      g += 0.9 * (c.g - g);
      b += 0.9 * (c.b - b);
      a += 0.9 * (c.a - a);
      return *this;
    }
  }

  rgb3 &operator+=(const rgb3 &c) {
    *this = *this + c;
    return *this;
  }

  bool operator==(const rgb3 &rhs) const {
    return (r == rhs.r) && (g == rhs.g) && (b == rhs.b);
  }

  bool operator!=(const rgb3 &rhs) const { return !((*this) == rhs); }
};
