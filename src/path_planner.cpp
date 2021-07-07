#include "path_planner.hpp"

#include<set>
#include<iostream>
#include<cassert>
#include<queue>
#include<math.h>

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
      if(out[i].kind == GenKind::start ){
        starts.push_back(&out[i]);
      }else if(out[i].kind == GenKind::target){
        ends.push_back(&out[i]);
      }
    }
    connect(out, xs.size(), ys.size());
  }

  void graph_from_poly(
   std::vector<struct area_t> &in, 
   std::vector<struct node_t> &out,
   std::vector<struct node_t*> &starts,
   std::vector<struct node_t*> &ends){
    std::vector<int> xs;
    std::vector<int> ys;
    get_grid(xs, ys, in);

    grid_to_graph(xs, ys, in, out, starts, ends);
  }
  int get_dst(const path_t &p){
    return 0;
  }
  
  int ccost(node_t *a, node_t *b){
    int cax, cay, cbx, cby;
    cax = (a->x + a->Mx) / 2;
    cay = (a->y + a->My) / 2;
    cbx = (b->x + b->Mx) / 2;
    cby = (b->y + b->My) / 2;
    return (int)sqrt(pow((cax - cbx), 2) + pow(cay - cby, 2));
  }

  void update_cost(path_t &p){
    size_t len = p.nodes.size();
    node_t *prec = p.nodes[len - 2];
    node_t *now = p.nodes[len - 1];
    p.cost += ccost(prec, now);
    if(p.nodes.back()->kind == GenKind::victim){
    }
  }
  
  void path_append(path_t &p, node_t *n){
    p.nodes.push_back(n);
    update_cost(p);
  }

  void clean(std::vector<node_t> &graph){
    for(int i = 0; i<graph.size(); i++){
      graph[i].cost = 1e9;
    }
  }

  void plan(std::vector<struct node_t> &graph, node_t *start, path_t &best){
    clean(graph);
    auto cmp = [](const path_t &left, const path_t &right) {
      int dst_l = get_dst(left);
      int dst_r = get_dst(right);
      int cst_l = left.cost;
      int cst_r = right.cost;
      int cumul_l = dst_l + cst_l;
      int cumul_r = dst_r + cst_r;
      return cumul_l == cumul_r ? cst_l > cst_r : cumul_l > cumul_r;
    };
    std::priority_queue<path_t, std::vector<path_t>, decltype(cmp)> pq(cmp);
    path_t start_p = {0,{}, std::vector<node_t*>({start})};
    best.cost = 1e9;
    pq.push(start_p);
    while(!pq.empty()){
      path_t current = pq.top();
      pq.pop();
      for(node_t *nei : current.nodes.back()->nei){
        path_t new_path = current;//copy hopefully
        path_append(new_path, nei);
        if(nei->kind == GenKind::target){
          best = new_path;
        }else if(new_path.cost < nei->cost &&
         new_path.cost < best.cost){
          nei->cost = new_path.cost;
          pq.push(new_path);
        }
      }
    }
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
  /*a.push_back({GenKind::obstacle, 0, 0, 1, 1, 1});
  a.push_back({GenKind::victim, 1, 1, 2, 2, 1});
  a.push_back({GenKind::target, 0, 1, 1, 2, -1});
  a.push_back({GenKind::start, 1, 0, 2, 1, -1});
  */
  vector<node_t> t;
  vector<node_t*> starts;
  vector<node_t*> ends;
  path_planner::graph_from_poly(a, t, starts, ends);
  for(auto n : t){
    std::cout << n.x << " : " << n.y << " > " << std::endl;
    for(auto n1 : n.nei){
      std::cout << " " << n1->x << " : " << n1->y << std::endl;
    }
  }
}
#endif