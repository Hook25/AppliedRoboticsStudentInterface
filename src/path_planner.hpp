#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include<vector>
#include<set>
#include<complex>

namespace path_planner{
  enum class GenKind{
    uncat, free, obstacle, victim, target, start
  };
  struct area_t {
    GenKind kind;
    double x;
    double y;
    double Mx;
    double My;
    int vic_id;
  };
  struct node_t{
    double x;
    double y;
    double Mx;
    double My;
    double cx; 
    double cy;
    int vic_id;
    GenKind kind;
    std::vector<struct node_t*> nei;
    double cost;
    double dst_target;
  };
  struct path_t{
    double cost;
    std::set<int> collected_victims;
    std::vector<struct node_t*> nodes;
  };
  void graph_from_poly(
    std::vector<struct area_t> &in, 
    std::vector<struct node_t> &out,
    std::vector<struct node_t*> &starts,
    std::vector<struct node_t*> &ends
  );
  void plan(
    std::vector<struct node_t> &graph, 
    node_t *start, 
    path_t &best, 
    double tx, 
    double ty
  );
  void build_smooth_path(
    const path_t &input, 
    std::vector<std::complex<double>> &output
  );
     
}
#endif