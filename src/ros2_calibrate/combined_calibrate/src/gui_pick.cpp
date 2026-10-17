#include "combined_calibrate/gui_pick.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace combined_calibrate {
namespace {

struct PickState {
  cv::Mat display;
  double scale = 1.0;
  int count = 4;
  std::vector<std::string> labels;
  std::vector<cv::Point2d> display_points;
  std::vector<cv::Point2d> image_points;
  bool dirty = true;
};

std::string labelFor(const std::vector<std::string> &labels, size_t index) {
  if (index < labels.size() && !labels[index].empty()) return labels[index];
  return "P" + std::to_string(index + 1);
}

void drawStatus(cv::Mat &canvas, const PickState &state) {
  const std::string progress =
      "picked " + std::to_string(state.image_points.size()) + "/" + std::to_string(state.count);
  const std::string help =
      "left=add  right=undo  u=undo  r=reset  ENTER=accept  ESC=abort   order: ";
  std::string order;
  for (int i = 0; i < state.count; ++i) {
    if (i != 0) order += " -> ";
    order += labelFor(state.labels, size_t(i));
  }
  const int line_height = 26;
  const int bars = 3;
  cv::rectangle(canvas, cv::Rect(0, 0, canvas.cols, line_height * bars + 8), cv::Scalar(0, 0, 0),
                cv::FILLED);
  cv::putText(canvas, progress, cv::Point(8, line_height - 6), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 255, 255), 2, cv::LINE_AA, false);
  cv::putText(canvas, help, cv::Point(8, line_height * 2 - 6), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(200, 200, 200), 1, cv::LINE_AA, false);
  cv::putText(canvas, order, cv::Point(8, line_height * 3 - 6), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(120, 255, 120), 1, cv::LINE_AA, false);
}

void onMouse(int event, int x, int y, int flags, void *userdata) {
  auto *state = static_cast<PickState *>(userdata);
  if (state == nullptr) return;
  if (event == cv::EVENT_LBUTTONDOWN) {
    if (static_cast<int>(state->display_points.size()) >= state->count) return;
    state->display_points.emplace_back(x, y);
    state->image_points.emplace_back(double(x) / state->scale, double(y) / state->scale);
    state->dirty = true;
    std::cout << "  " << labelFor(state->labels, state->image_points.size() - 1) << " = ("
              << state->image_points.back().x << ", " << state->image_points.back().y << ")" << std::endl;
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    if (!state->display_points.empty()) {
      state->display_points.pop_back();
      state->image_points.pop_back();
      state->dirty = true;
    }
  }
  (void)flags;
}

}  // namespace

cv::Mat drawCornerAnnotations(const cv::Mat &bgr, const std::vector<cv::Point2d> &points,
                              const std::vector<std::string> &labels) {
  cv::Mat canvas = bgr.clone();
  for (size_t i = 0; i < points.size(); ++i) {
    const cv::Point center(cvRound(points[i].x), cvRound(points[i].y));
    cv::drawMarker(canvas, center, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 28, 2);
    cv::circle(canvas, center, 6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    const std::string text = labelFor(labels, i);
    const cv::Point origin = center + cv::Point(12, -12);
    cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 5,
                cv::LINE_AA, false);
    cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2,
                cv::LINE_AA, false);
  }
  return canvas;
}

PickOutcome pickCornersOnImage(const cv::Mat &bgr, int count, const std::vector<std::string> &labels,
                               const std::string &window_title, int max_width) {
  PickOutcome outcome;
  if (bgr.empty() || count <= 0) {
    outcome.aborted = true;
    return outcome;
  }

  PickState state;
  state.count = count;
  state.labels = labels;
  state.scale = 1.0;
  if (max_width > 0 && bgr.cols > max_width) {
    state.scale = double(max_width) / double(bgr.cols);
    cv::resize(bgr, state.display, cv::Size(), state.scale, state.scale, cv::INTER_AREA);
  } else {
    state.display = bgr.clone();
  }

  bool window_created = false;
  try {
    cv::namedWindow(window_title, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_title, onMouse, &state);
    window_created = true;
  } catch (const cv::Exception &exception) {
    std::cerr << "cannot open the picking window (" << exception.what()
              << "), falling back to headless mode" << std::endl;
    outcome.aborted = true;
    return outcome;
  }

  while (true) {
    if (state.dirty) {
      cv::Mat canvas = state.display.clone();
      for (size_t i = 0; i < state.display_points.size(); ++i) {
        const cv::Point center(cvRound(state.display_points[i].x), cvRound(state.display_points[i].y));
        cv::drawMarker(canvas, center, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 20, 2);
        const std::string text = labelFor(state.labels, i);
        cv::putText(canvas, text, center + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 0, 0), 4, cv::LINE_AA, false);
        cv::putText(canvas, text, center + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 0), 2, cv::LINE_AA, false);
      }
      drawStatus(canvas, state);
      cv::imshow(window_title, canvas);
      state.dirty = false;
    }

    const int key = cv::waitKey(20);
    if (key == 27) {
      outcome.aborted = true;
      break;
    }
    if (key == 13 || key == 10 || key == ' ') {
      if (state.image_points.empty()) {
        std::cout << "no corner picked yet" << std::endl;
        continue;
      }
      outcome.accepted = true;
      outcome.points = state.image_points;
      break;
    }
    if (key == 'u' || key == 'U') {
      if (!state.image_points.empty()) {
        state.image_points.pop_back();
        state.display_points.pop_back();
        state.dirty = true;
      }
      continue;
    }
    if (key == 'r' || key == 'R') {
      state.image_points.clear();
      state.display_points.clear();
      state.dirty = true;
      continue;
    }

    if (window_created) {
      double visible = 1.0;
      try {
        visible = cv::getWindowProperty(window_title, cv::WND_PROP_VISIBLE);
      } catch (const cv::Exception &exception) {
        visible = 1.0;
      }
      if (visible == 0.0) {
        outcome.aborted = true;
        break;
      }
    }
  }

  if (window_created) {
    try {
      cv::destroyWindow(window_title);
    } catch (const cv::Exception &exception) {
      // nothing to clean up
    }
  }
  if (outcome.accepted && static_cast<int>(outcome.points.size()) != count) {
    std::cout << "warning: picked " << outcome.points.size() << " corners, expected " << count
              << std::endl;
  }
  return outcome;
}

}  // namespace combined_calibrate
