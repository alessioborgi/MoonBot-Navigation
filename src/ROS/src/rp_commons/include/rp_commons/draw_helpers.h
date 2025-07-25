#pragma once
#include <opencv2/opencv.hpp>

#include "Eigen/Core"
#include "grid.h"

using Canvas = cv::Mat;

/**
 * @brief Draws a grid on a canvas.
 *
 * @param dest Destination canvas.
 * @param src Source grid.
 * @param free Value of free cells.
 * @param occupied Value of occupied cells.
 * @param unknown Value of unknown cells.
 */
void drawGrid(Canvas& dest, const Grid_<uint8_t>& src, const unsigned char free,
              const unsigned char occupied, const unsigned char unknown);

/**
 * @brief Draws a line on a canvas.
 *
 * @param dest Destination canvas.
 * @param p0 Start point.
 * @param p1 End point.
 * @param color Color of the line.
 */
void drawLine(Canvas& dest, const Eigen::Vector2i& p0,
              const Eigen::Vector2i& p1, uint8_t color);

/**
 * @brief Draws a circle on a canvas.
 *
 * @param dest Destination canvas.
 * @param center Center of the circle.
 * @param radius Radius of the circle.
 * @param color Color of the circle.
 */
void drawCircle(Canvas& dest, const Eigen::Vector2i& center, int radius,
                uint8_t color);

/**
 * @brief Shows a canvas and waits for a key press.
 *
 * @param canvas Canvas to show.
 * @param timeout_ms Timeout in milliseconds.
 * @return Key pressed.
 */
int showCanvas(Canvas& canvas, int timeout_ms);
