#include "rp_commons/draw_helpers.h"

void drawGrid(Canvas& dest, const Grid_<uint8_t>& src, const unsigned char free,
              const unsigned char occupied, const unsigned char unknown) {
  dest = cv::Mat(src._rows, src._cols, CV_8UC1);
  for (size_t i = 0; i < src._rows; ++i) {
    for (size_t j = 0; j < src._cols; ++j) {
      uint8_t value = src._cells[i * src._cols + j];
      if (value == free) {
        dest.at<uint8_t>(i, j) = 255;
      } else if (value == occupied) {
        dest.at<uint8_t>(i, j) = 0;
      } else if (value == unknown) {
        dest.at<uint8_t>(i, j) = 200;
      } else {
        dest.at<uint8_t>(i, j) = value;  // Preserve other values
      }
    }
  }
}

void drawLine(Canvas& dest, const Eigen::Vector2i& p0,
              const Eigen::Vector2i& p1, uint8_t color) {
  cv::line(dest, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]),
           cv::Scalar(color, color, color), 1);
}

void drawCircle(Canvas& dest, const Eigen::Vector2i& center, int radius,
                uint8_t color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius,
             cv::Scalar(color, color, color));
}

int showCanvas(Canvas& canvas, int timeout_ms) {
  cv::imshow("canvas", canvas);
  int key = cv::waitKey(timeout_ms);
  if (key == 27)  // exit on ESC
    exit(0);
  // cerr << "key" << key << endl;
  return key;
}