#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include<vector>
#include<set>
namespace path_planner{
  enum class GenKind{
    uncat, free, obstacle, victim, target, start
  };
  struct area_t {
    GenKind kind;
    int x;
    int y;
    int Mx;
    int My;
    int vic_id;
  };
  struct node_t{
    int x;
    int y;
    int Mx;
    int My;
    int cx; 
    int cy;
    int vic_id;
    GenKind kind;
    std::vector<struct node_t*> nei;
    int cost;
    int dst_target;
  };
  struct path_t{
    int cost;
    std::set<int> collected_victims;
    std::vector<struct node_t*> nodes;
  };
  void graph_from_poly(std::vector<struct area_t> &in, std::vector<struct node_t> &out);
  void plan(std::vector<struct node_t> &graph, path_t &best);
}
#endif