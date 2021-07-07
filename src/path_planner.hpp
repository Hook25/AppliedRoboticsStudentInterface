#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include<vector>
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
    int vic_id;
    GenKind kind;
    std::vector<struct node_t*> nei;
  };
  void graph_from_poly(std::vector<struct area_t> &in, std::vector<struct node_t> &out);
}
#endif