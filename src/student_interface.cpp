#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <ctime>
#include <stdexcept>
#include <sstream>
#include <experimental/filesystem>

#define DISPLAY_SCALE 2.0
#define AL_EXT_CAL true

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#define DBG_SHOW(x) do{cv::imshow(__FUNCTION__, (x)); cv::waitKey(0);}while(0)
#endif

namespace fs = std::experimental::filesystem;

namespace student {

 void loadImage(cv::Mat& img_out, const std::string& config_folder){  
   throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    static char d_path[500];
    static fs::path s_path;
    if(*d_path){ //TODO: this is not thread safe
      time_t rawtime;
      struct tm * timeinfo;
      time (&rawtime);
      timeinfo = localtime(&rawtime);
      //TODO: will break if res is > than 500 char
      strftime(d_path, sizeof(d_path), "%d%m%Y_%H%M%S", timeinfo);
      s_path = config_folder;
      s_path /= "camera_image";
      s_path /= d_path;
      if(!fs::create_directories(s_path.u8string())){
        std::cout << "Using directory that existed: " << s_path.u8string() << std::endl;
      }
    }
    cv::imshow(topic, img_in);
    char c = cv::waitKey(100);
    if(c == 's'){
      struct timespec monotime;
      clock_gettime(CLOCK_MONOTONIC, &monotime); 
      std::stringstream tmp;
      tmp << monotime.tv_sec << monotime.tv_nsec;
      cv::imwrite((s_path / tmp.str()).u8string(), img_in);
      std::cout << "Saved: " <<  ((s_path / tmp.str()).u8string()) << std::endl;
      
    }
  }
 
  struct cb_arg {
    cv::Mat *img;
    std::vector<cv::Point2f> *points;
    int n;
    std::string *name;
  };

  void async_picker_mousecb(int event, int x, int y, int k, void *p){
    cb_arg *in = static_cast<cb_arg*>(p); //TODO: dogshit ub
    if(event != cv::EVENT_LBUTTONDOWN || in->points->size() >= in->n){ return; }
    in->points->emplace_back(x*DISPLAY_SCALE, y*DISPLAY_SCALE);
    cv::circle(*in->img, cv::Point(x,y), 20/DISPLAY_SCALE, cv::Scalar(0, 0, 255), -1);
    cv::imshow(in->name->c_str(), *in->img);        
  }

  void get_points(int count, const cv::Mat &img, std::vector<cv::Point2f> *points){
    points->clear();
    cv::Size scale(img.cols/DISPLAY_SCALE, img.rows/DISPLAY_SCALE);
    cv::Mat scaled_img;
    cv::resize(img, scaled_img, scale);
    std::string name = "Pick " + std::to_string(count);
    cv::imshow(name.c_str(), scaled_img);
    cv::namedWindow(name.c_str());
    cb_arg arg = { &scaled_img, points, count, &name };
    cv::setMouseCallback(name.c_str(), &async_picker_mousecb, &arg);
    while(points->size() < count){
      cv::waitKey(500);
    } 
    cv::destroyWindow(name.c_str());
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    fs::path csv_path(config_folder);
    csv_path /= "extrinsicCalib.csv";
    std::ifstream input(csv_path);

    static std::vector<cv::Point2f> points;

    if(input.is_open() && AL_EXT_CAL){
      //TL TR BR BL you fucking animal
      while(!input.eof()){
        double x,y;
        input >> x >> y; //TODO: remember no nl at eof
        points.emplace_back(x,y);
      }
      input.close();
    } else {
      get_points(4, img_in, &points);
      std::ofstream out(csv_path);
      bool first = true;
      for(const auto pt : points){
        if(!first){ out << std::endl; }
        first = false;
        out << pt.x << " " << pt.y;
      }
      out.close();
    }
    cv::Mat coeff;
    coeff = (cv::Mat1d(1,4) << 0,0,0,0,0);
    return cv::solvePnP(object_points, points, camera_matrix, coeff, rvec, tvec);
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){

    cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
    cv::Mat img_points;
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), img_points);
    plane_transf = cv::getPerspectiveTransform(img_points, dest_image_points_plane); 
  }


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    DBG_SHOW(img_in);
    std::cout << "transform matrix: " << transf << std::endl;
    cv::warpPerspective(img_in, img_out, transf, img_in.size());  
    DBG_SHOW(img_out);
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    cv::imshow("MAP", img_in);
    cv::waitKey(0);
    throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );   
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    cv::imshow("MAP", img_in);
    cv::waitKey(0);
    throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );    
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );     
  }


}

