#include "path_planner.hpp"

#include<set>
#include<iostream>
#include<cassert>

namespace path_planner {
  void sorted_nodup(std::vector<int> &v){
    std::set<int> s(v.begin(), v.end());
    v.assign( s.begin(), s.end() );
  }
  void get_grid(std::vector<int> &xs, std::vector<int> &ys, std::vector<struct area_t> &in){
    xs.push_back(0);
    ys.push_back(0);

    for(auto a : in){
      xs.push_back(a.x);
      xs.push_back(a.Mx);
      ys.push_back(a.y);
      ys.push_back(a.My);
    }
    sorted_nodup(xs);
    sorted_nodup(ys);
  }

  inline bool intersects(const area_t &a, const node_t &n){
    //node inside (or eq) area 
    return n.x >= a.x && n.y >= a.y && n.x < a.Mx && n.y < a.My;
  }

  void categorize(const std::vector<struct area_t> areas, node_t &n){
    for(auto a : areas){
      if(intersects(a, n)) {
        n.kind = a.kind;
        n.vic_id = a.vic_id;
        return;
      }
    }
    n.kind = GenKind::free;
  }

  inline node_t* at(std::vector<struct node_t> &v, int x, int y, int size_y){
    int pos = x*size_y + y;
    node_t *to_r = 0;
    if(x >= 0 && y >= 0 && pos < v.size() && y < size_y){
      to_r = &v[pos];
    }
    return to_r;
  }

  inline bool is_obst(const node_t *n){
    return (!n) || (n->kind == GenKind::obstacle); 
  }

  void connect(std::vector<struct node_t> &ns, int max_x, int max_y){
    int x = 0;
    int y = 0;
    int comp_y = max_y - 1;
    for(int i = 0; i < ns.size(); i++){
      if(ns[i].kind != GenKind::obstacle){
        node_t *nei[4] = {
          at(ns, x-1, y  , comp_y), 
          at(ns, x+1, y  , comp_y), 
          at(ns, x  , y-1, comp_y), 
          at(ns, x  , y+1, comp_y)
        };
        for(int nei_i = 0; nei_i < 4; nei_i++){
          if(!is_obst(nei[nei_i])){
            ns[i].nei.push_back(nei[nei_i]);
          }
        }
      }
      y = (y + 1) % comp_y;
      if(y == 0){
        x ++;
      }
    }
  }
  void grid_to_graph(
    const std::vector<int> &xs,
    const std::vector<int> &ys, 
    const std::vector<struct area_t> &in,
    std::vector<struct node_t> &out, 
    std::vector<struct node_t*> &starts, 
    std::vector<struct node_t*> &ends)
  {
    auto x_bound = xs.end() - 1;
    auto y_bound = ys.end() - 1;
    //create a node per coord couple
    for (auto x = xs.begin(); x != x_bound; x++){
      for(auto y = ys.begin(); y != y_bound; y++){
        node_t n = {*x,  *y, *(x+1), *(y+1)};
        out.push_back(n);
      }
    }
    //give kind and id to each node
    for(int i = 0; i < out.size(); i++){
      categorize(in, out[i]);
      if(out[i].kind == GenKind::start || ){
        starts.push_back(&out[i]);
      }elif(out[i].kind == GenKind::target){
        ends.push_back(&out[i]);
      }
    }
    connect(out, xs.size(), ys.size());
  }

  void graph_from_poly(std::vector<struct area_t> &in, std::vector<struct node_t> &out){
    std::vector<int> xs;
    std::vector<int> ys;
    get_grid(xs, ys, in);

    grid_to_graph(xs, ys, in, out);
  }
}

#ifdef EXAMPLE_USAGE
using namespace path_planner;
using namespace std;
/*
example = [
  [obstacle], [target  ],
  [start   ], [victim  ]
]
*/
int main(void){
  vector<area_t> a;
  for(int x = 0; x < 20; x++){
    for(int y = 0; y < 20; y++){
      a.push_back({GenKind::victim, x, y, x+1, y+1, -1});
    }
  }
  /*a.push_back({GenKind::victim, 1, 1, 2, 2, 1});
  a.push_back({GenKind::target, 0, 1, 1, 2, -1});
  a.push_back({GenKind::start, 1, 0, 2, 1, -1});
  */
  vector<node_t> t;
  path_planner::graph_from_poly(a, t);
  for(auto n : t){
    std::cout << n.x << " : " << n.y << " > " << std::endl;
    for(auto n1 : n.nei){
      std::cout << " " << n1->x << " : " << n1->y << std::endl;
    }
  }
}
#endif