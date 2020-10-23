#ifndef MAP_WATCHER_H
#define MAP_WATCHER_H

#include "student_image_elab_interface.hpp"
#include<string.h>

namespace map_watcher {
  struct n_template{
    int id;
    cv::Mat img;
  };
  struct victim {
    cv::Point loc;
    std::vector<cv::Point> shape;
    int rad;
    int id;
  };
  struct robot{
    std::vector<cv::Point> cont;
    cv::Point loc;
    cv::Point tail_loc;
    double angle;
  };
  void init(const std::string &conf_dir_path);
  void findVictimsGate(const cv::Mat &img, std::vector<cv::Point> &gate, std::vector<victim> &victims);
  void findObstacles(const cv::Mat &img, std::vector<std::vector<cv::Point>> &conts);
  void findRobot(const cv::Mat &img, robot &r);
}
#endif
