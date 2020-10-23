#include "map_watcher.hpp"
#include<stdlib.h> 
#include<experimental/filesystem>
#include<iostream>  
#include<glob.h>

#define TEMPLATES_PATH "templates"

#define KERNEL_COL_DCT_SIZE cv::Size(3,3)

#define GREEN_LOW cv::Scalar(54, 127, 0)
#define GREEN_HIGH cv::Scalar(98, 255, 255)

#define RED_LOW cv::Scalar(0, 171, 0)
#define RED_HIGH cv::Scalar(43, 255,255)

#define BLUE_LOW cv::Scalar(99, 180, 0)
#define BLUE_HIGH cv::Scalar(123, 240, 255)

#define VICTIMS_TH_SIZE .3
#define VICTIMS_TH_APPROX .01

#define OBSTACLES_TH_APPROX .03
#define OBSTACLES_TH_SIZE .01

namespace fs = std::experimental::filesystem;

namespace map_watcher {
  static std::string _conf_dir_path;
  void init(const std::string &conf_dir_path){
    _conf_dir_path = conf_dir_path;
  }
  int _getTemplateIndex(char *name){
    //name is ...img(num).jpg
    char *token = strtok(name, "g");
    char *next = token;
    while(next != NULL){
      next = strtok(0, "g"); //goto the last g in path
      if(next){
        token = next;
      }
    }
    //token is now: (num).jpg
    next = strtok(token, ".");
    assert(next);
    return atoi(token);
  }
  void loadNumTemplates(std::vector<n_template> *storage){
    fs::path path; 
    path = _conf_dir_path;
    path /= TEMPLATES_PATH;
    path /= "*.jpg";
    auto data_p = path.u8string();
    glob_t pglob;
    int err = glob(data_p.data(), GLOB_ERR, 0, &pglob);
    if(!err && pglob.gl_pathc){
      for(int i = 0; i<pglob.gl_pathc; i++){
        char *str = pglob.gl_pathv[i];
        auto res = cv::imread(str);
        int tag = _getTemplateIndex(str);
        storage->push_back({tag, res});
      }
      globfree(&pglob);
    }else{
      throw std::runtime_error(std::string("No file matches ") + path.u8string());
    } 
  }
  void findCol(const cv::Mat &img_hsv, cv::Scalar lower, cv::Scalar upper, 
   std::vector<std::vector<cv::Point>> &conts){
    cv::Mat mask;
    cv::inRange(img_hsv, lower, upper, mask);
    auto kernel = cv::getStructuringElement(cv::MORPH_RECT, KERNEL_COL_DCT_SIZE);
    cv::Mat opening;
    cv::morphologyEx(mask, opening, cv::MORPH_CLOSE, kernel);
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(opening, conts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  }
  double pointDst(cv::Point p1, cv::Point p2){
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
  }
  void approxCont(std::vector<cv::Point> &cont, double param, std::vector<cv::Point> &out){
    auto epsilon = param * cv::arcLength(cont, true);
    cv::approxPolyDP(cont, out,  epsilon, true);
  }
  void approxConts(std::vector<std::vector<cv::Point>> &conts, 
   double th, std::vector<std::vector<cv::Point>> &out){
    out.resize(conts.size());
    for(int i = 0; i<conts.size(); i++){
      approxCont(conts[i], th, out[i]);
    }
  }
  void filterBySize(std::vector<std::vector<cv::Point>> &conts, double factor){
    double max_size_d = cv::contourArea(conts[0]);
    for(auto cont : conts){
      double cont_size = cv::contourArea(cont);
      if(cont_size > max_size_d){
        max_size_d = cont_size;
      }
    }
    double min_size = max_size_d * factor;
    conts.erase(std::remove_if(conts.begin(), conts.end(), 
      [min_size](std::vector<cv::Point> &c) -> bool
    {
      return cv::contourArea(c) < min_size;
    }), conts.end());
  }
  void getGate(std::vector<std::vector<cv::Point>> &conts, std::vector<cv::Point> &gate){
    int index = 0;
    int min_edg = conts[0].size();
    for(int i = 0; i<conts.size(); i++){
      if(conts[i].size() < min_edg){
        min_edg = conts[i].size();
        index = i;
      }
    }
    cv::RotatedRect rr = cv::minAreaRect(conts[index]);
    cv::Mat x;
    cv::boxPoints(rr, x);
    //TODO: this is beyond retarded and getting worse
    for(int point = 0; point < 4; point++){
      //boxPoint ret is a Mat for some reason with (x, y) pairs as rows
      gate.emplace_back(cv::Point2f(x.at<float>(point, 0), x.at<float>(point,1)));
    } 
    //remove the gate from green items
    conts.erase(conts.begin() + index);
  }
  int templateMatching(const cv::Mat &img){
    static std::vector<n_template> storage;
    if(storage.size() == 0){
     loadNumTemplates(&storage);
     for(auto s : storage){
       std::cout << s.id << std::endl; 
     }
    }
    auto methods = {cv::TM_CCOEFF_NORMED, cv::TM_CCORR_NORMED};
    double best_cmp_val = 0;
    double best_cmp_i = 0;
    for(int i = 0; i<storage.size(); i++){
      cv::Mat loc_img = img.clone();
      cv::Mat res;
      for(auto method : methods){
        cv::matchTemplate(loc_img, storage[i].img, res, method);
        cv::Point min_p, max_p;
        double min_v, max_v;
        cv::minMaxLoc(res, &min_v, &max_v, &min_p, &max_p);
        double cmp_val = max_v;
        if(method == cv::TM_SQDIFF_NORMED){
          cmp_val = (1-min_v);
        }
        if(cmp_val > best_cmp_val){
          best_cmp_val = cmp_val;
          best_cmp_i = i;
        }
      }
    }
    return storage[best_cmp_i].id;
  }
  int identifyVictim(const cv::Mat &img, double x, double y, double rad){
    //TODO: this could be way faster by first cutting then masking, consider changing
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8U);
    cv::circle(mask, cv::Point(x,y), rad, cv::Scalar(255,255,255), CV_FILLED, 8, 0);
    cv::Mat masked;
    img.copyTo(masked, mask);
    cv::Mat loc(masked, cv::Rect(x-rad, y-rad, 2*rad, 2*rad));
    cv::cvtColor(loc, loc, cv::COLOR_HSV2BGR);
    //TODO: prob we can afford to rotate loc? if bad check
    return templateMatching(loc);
  }
  void getVictims(const cv::Mat &img, const std::vector<std::vector<cv::Point>> &conts, 
   std::vector<victim> &victims){
    victims.resize(conts.size());
    for(int i = 0; i<conts.size(); i++){
      cv::Point2f rt_val;//no reason in the whole world this has to be Point2f
      float rad;         //float ofc, as above
      minEnclosingCircle(conts[i], rt_val, rad);
      victims[i].loc = rt_val;
      victims[i].rad = rad;
      victims[i].id = identifyVictim(img, victims[i].loc.x, victims[i].loc.y, victims[i].rad);
    }
  }
  cv::Point getCentre(const std::vector<cv::Point> &shape){
    double x = 0;
    double y = 0;
    for(auto point : shape){
      x+=point.x;
      y+=point.y;
    }
    return cv::Point2f(x/shape.size(), y/shape.size());
  } 
  void findVictimsGate(const cv::Mat &img, std::vector<cv::Point> &gate, std::vector<victim> &victims){
    std::vector<std::vector<cv::Point>> conts;
    findCol(img, GREEN_LOW, GREEN_HIGH, conts);
    filterBySize(conts, VICTIMS_TH_SIZE);
    approxConts(conts, VICTIMS_TH_APPROX, conts);
    /*cv::drawContours(img, conts, -1, cv::Scalar(0,0,0), 3);
    cv::imshow("tmp", img);
    cv::waitKey(0);
    */
    getGate(conts, gate);
    getVictims(img, conts, victims);
  }
  void findObstacles(const cv::Mat &img, std::vector<std::vector<cv::Point>> &conts){
    findCol(img, RED_LOW, RED_HIGH, conts);
    approxConts(conts, OBSTACLES_TH_APPROX, conts);
    filterBySize(conts, OBSTACLES_TH_SIZE);
  }
  void findRobot(const cv::Mat &img, robot &r){
    std::vector<std::vector<cv::Point>> loc;
    findCol(img, BLUE_LOW, BLUE_HIGH, loc);
    filterBySize(loc, 1); //returns 1 given that 1 means only eq bigger
    cv::minEnclosingTriangle(loc[0], r.cont);
    r.loc = getCentre(r.cont);
    double max_dist = 0;
    for(auto point : r.cont){
      double dist = pointDst(point, r.loc);
      if(dist > max_dist){
        max_dist = dist;
        r.tail_loc = point;
      }
    }
    r.angle = atan2(r.loc.y - r.tail_loc.y, r.loc.x - r.tail_loc.x);
  }
}
#ifdef EXAMPLE_USAGE
int main(void){
  map_watcher::init("/tmp");
  std::vector<cv::Point> gate;
  std::vector<map_watcher::victim> victims;
  std::vector<std::vector<cv::Point>> obstacles;
  auto img = cv::imread("img.jpg");
  auto img1 = img.clone();
  cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
  findVictimsGate(img, gate, victims);
  for(auto vic : victims){
    cv::circle(img1, vic.loc, vic.rad, cv::Scalar(0,0,0), 5);
    cv::putText(img1, std::to_string(vic.id), vic.loc, cv::FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 0), 2);
  }
  map_watcher::robot bot;
  findRobot(img, bot);
  cv::circle(img1, bot.loc, 3, (255,255,255), -1);
  map_watcher::findObstacles(img, obstacles);
  std::cout << "tot obstacles" << obstacles.size() << std::endl;
  obstacles.push_back(gate);
  obstacles.push_back(bot.cont);
  cv::drawContours(img1, obstacles, -1, (0,0,0), 5);
  cv::circle(img1, bot.tail_loc, 3, (255,255,255), -1);
  cv::imshow("ABC", img1);
  cv::waitKey(0);
}
#endif
