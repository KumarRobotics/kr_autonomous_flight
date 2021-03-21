#include <mpl_basis/trajectory.h>
#include <mpl_collision/map_util.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define black cv::Scalar(0, 0, 0)
#define grey cv::Scalar(100, 100, 100)
#define red cv::Scalar(0, 0, 255)
#define blue cv::Scalar(255, 0, 0)
#define green cv::Scalar(0, 255, 0)
#define cyan cv::Scalar(255, 255, 0)
#define magenta cv::Scalar(255, 0, 255)

/// Opencv drawing helper
class OpenCVDrawing {
 public:
  /// Simple constructor
  OpenCVDrawing(std::shared_ptr<MPL::OccMapUtil> map_util)
      : map_util_(map_util) {
    const auto dim = map_util->getDim();
    // Create black empty images
    img_ = cv::Mat(dim(1), dim(0), CV_8UC3, cv::Scalar(255, 255, 255));
  }

  /// Draw points
  void drawPoints(const vec_Vec2f &pts, cv::Scalar color, int line_width = 1) {
    for (const auto &it : pts) {
      const auto pn = map_util_->floatToInt(it);
      cv::Point pt(pn(0), pn(1));
      cv::rectangle(img_, pt, pt, color, line_width);
    }
  }

  /// Draw circle
  void drawCircle(const Vec2f &pt, cv::Scalar color, int r,
                  int line_width = 1) {
    const auto pn = map_util_->floatToInt(pt);
    cv::circle(img_, cv::Point(pn(0), pn(1)), r, color, line_width);
  }

  /// Draw trajectory, sample n points
  void drawTraj(const Trajectory2D &traj, cv::Scalar color, int line_width = 1,
                int num = 200) {
    const auto ws = traj.sample(num);
    for (size_t i = 0; i < ws.size() - 1; i++) {
      const auto pn1 = map_util_->floatToInt(ws[i].pos);
      const auto pn2 = map_util_->floatToInt(ws[i + 1].pos);
      cv::line(img_, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)),
               color, line_width);
    }
  }

  /// Draw text
  void drawText(std::string text, const Vec2i &pn, double scale,
                cv::Scalar color) {
    cv::putText(img_, text, cv::Point(pn(0), pn(1)), cv::FONT_ITALIC, scale,
                color, 2);
  }

  /// Draw line strip
  void drawLineStrip(const vec_E<vec_Vec2f> &trias, cv::Scalar color,
                     int line_width = 1) {
    for (const auto &it : trias) {
      for (size_t i = 0; i < it.size() - 1; i++) {
        const auto pn1 = map_util_->floatToInt(it[i]);
        const auto pn2 = map_util_->floatToInt(it[i + 1]);
        cv::line(img_, cv::Point(pn1(0), pn1(1)), cv::Point(pn2(0), pn2(1)),
                 color, line_width);
      }
    }
  }

  /// Draw potential map
  void drawPotential(cv::Scalar c1, cv::Scalar c2) {
    const auto dim = map_util_->getDim();
    const auto data = map_util_->getMap();
    for (int x = 0; x < dim(0); x++) {
      for (int y = 0; y < dim(1); y++) {
        Vec2i pn(x, y);
        decimal_t occ = (decimal_t)data[map_util_->getIndex(Vec2i(x, y))] / 100;
        if (occ <= 0 || occ >= 1) continue;
        cv::Scalar color(0, 0, 0);
        color[0] = occ * c1[0] + (1 - occ) * c2[0];
        color[1] = occ * c1[1] + (1 - occ) * c2[1];
        color[2] = occ * c1[2] + (1 - occ) * c2[2];
        cv::rectangle(img_, cv::Point(x, y), cv::Point(x, y), color, 1);
      }
    }
  }

  /// Show the image
  void show(std::string window_name) {
    cv::imshow(window_name, img_);
    cv::waitKey(0);
  }

  /// Save the image
  void save(std::string file_name) { cv::imwrite(file_name, img_); }

 private:
  /// Map util
  std::shared_ptr<MPL::OccMapUtil> map_util_;
  /// Image object
  cv::Mat img_;
};
