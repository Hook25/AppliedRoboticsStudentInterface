#include "path_planner.hpp"

#include<set>
#include<iostream>
#include<cassert>
#include<queue>
#include<math.h>
#include<complex>

#include <opencv2/core/types.hpp>
/*
namespace cv{
  template<> cv::Point cv::Point::operator*(const cv::Point p1);
  template<int T>
  cv::Point cv::Point::operator*(const cv::Point p1){
    return cv::Point(p1.x * this->x, p1.y * this->y);
  }
}*/

#define EPSILON .0001

namespace path_planner {

  void sorted_nodup(std::vector<double> &v){
    std::set<double> s(v.begin(), v.end());
    v.assign(s.begin(), s.end());
  }

  void get_grid(std::vector<double> &xs, std::vector<double> &ys, std::vector<struct area_t> &in){
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
    //node_t inside (or eq) area 
    return n.x >= a.x && n.y >= a.y && n.x < a.Mx && n.y < a.My;
  }
  //TODO: fix this shit, too slow 
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
    const std::vector<double> &xs,
    const std::vector<double> &ys, 
    const std::vector<struct area_t> &in,
    std::vector<struct node_t> &out, 
    std::vector<struct node_t*> &starts, 
    std::vector<struct node_t*> &ends)
  {
    auto x_bound = xs.end() - 1;
    auto y_bound = ys.end() - 1;
    //create a node_t per coord couple
    for (auto x = xs.begin(); x != x_bound; x++){
      for(auto y = ys.begin(); y != y_bound; y++){
        node_t n = {*x,  *y, *(x+1), *(y+1)};
        out.push_back(n);
      }
    }
    //give kind and id to each node_t
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
    std::vector<double> xs;
    std::vector<double> ys;
    get_grid(xs, ys, in);

    grid_to_graph(xs, ys, in, out, starts, ends);
  }
  int get_dst(const path_t &p){
    return p.nodes.back()->dst_target;
  }
  
  double ccost(node_t *a, node_t *b){
    double cax, cay, cbx, cby;
    cax = (a->x + a->Mx) / 2;
    cay = (a->y + a->My) / 2;
    cbx = (b->x + b->Mx) / 2;
    cby = (b->y + b->My) / 2;
    return sqrt(pow((cax - cbx), 2) + pow(cay - cby, 2));
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

  void clean(std::vector<node_t> &graph, double tx, double ty){
    for(int i = 0; i<graph.size(); i++){
      graph[i].cost = 1e9;
      graph[i].cx = (graph[i].x + graph[i].Mx)/2;
      graph[i].cy = (graph[i].y + graph[i].My)/2;
      graph[i].dst_target = sqrt(pow(graph[i].cx - tx, 2) + pow(graph[i].cy - ty, 2));
    }
  }

  void plan(std::vector<struct node_t> &graph, node_t *start, path_t &best, double tx, double ty){
    clean(graph, tx, ty);
    auto cmp = [](const path_t &left, const path_t &right) {
      double dst_l = get_dst(left);
      double dst_r = get_dst(right);
      double cst_l = left.cost;
      double cst_r = right.cost;
      double cumul_l = dst_l;// + cst_l;
      double cumul_r = dst_r;// + cst_r;
      return cumul_l == cumul_r ? cst_l < cst_r : cumul_l < cumul_r;
    };
    std::priority_queue<path_t, std::vector<path_t>, decltype(cmp)> pq(cmp);
    path_t start_p = {0,{}, std::vector<node_t*>({start})};
    best.cost = 1e9;
    pq.push(start_p);
    while(!pq.empty()){
      path_t current = pq.top();
      pq.pop();
      for(node_t *nei : current.nodes.back()->nei){
        path_t new_path = current;//copy hopefullyst
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
  #define A 1
  inline double d_p(const std::vector<std::complex<double>> &p, int i){
    return pow(abs(p[i] - p[i-1]), A);
  }

  void calc_intermediates(const std::vector<std::complex<double>> &in, int lb, std::vector<std::complex<double>> &out){
    std::complex<double> p[] = {
      in[lb], in[lb + 1], in[lb + 2], in[lb + 3]
    };
    double d[] = {
      d_p(in, lb + 1), d_p(in, lb + 2), d_p(in, lb + 3)
    };
    double d_sq[] = {
      pow(d[0], 2), pow(d[1], 2), pow(d[2], 2)
    };
    std::complex<double> t1_top = (p[2] * d_sq[0] - p[0] * d_sq[1] + p[1] * (d_sq[0] * 2 + d[0] * 3 * d[1] + d_sq[1]));
    double t1_bot = (d[0] * 3 * (d[0] + d[1]));

    std::complex<double> t2_top = (p[1] * d_sq[2] - p[3] * d_sq[1] + p[2] * (d_sq[2] * 2 + d[2] * 3 * d[1] + d_sq[1]));
    double t2_bot = (d[2] * 3 * (d[2] + d[1]));
    std::complex<double> t1 = (
       t1_top / t1_bot
    );
    std::complex<double> t2 = (
       t2_top / t2_bot
    );
    out.push_back(p[1]);
    out.push_back(t1);
    out.push_back(t2);
    out.push_back(p[2]);

  }

  void find_tangents(const std::vector<node_t*> &in, std::vector<std::complex<double>> &out){
    std::vector<std::complex<double>> in_p;
    for(auto n_s : in){
      in_p.push_back(std::complex<double>(n_s->cx, n_s->cy));
    }
    //Disgusting hack but basically, necessary to have also the arrival and the start
    auto first = in_p.begin();
    in_p.insert(in_p.begin(), (*first + EPSILON * (*first - *(first+1))));
    auto last = in_p.end() - 1;
    in_p.push_back(*last + EPSILON * (*last - *(last - 1)));
    for(int i = 0; i < in_p.size() - 4 + 1; i++){
      calc_intermediates(in_p, i, out);
    }
  }

  void bezier_point(const std::vector<std::complex<double>> &in, int in_index, double t, std::vector<std::complex<double>> &out){
    std::vector<std::complex<double>> buffer(in.begin() + in_index, in.begin() + in_index + 4);
    while(buffer.size() > 1){
      std::vector<std::complex<double>> replacement;
      auto lw = buffer.begin();
      auto up = buffer.begin() + 1; 
      while(up != buffer.end()){
        replacement.push_back((*lw) * (1 - t) + (*up) * t);
        lw ++;
        up ++;
      }
      buffer = replacement;
    }
    if(buffer.size() == 1){
      out.push_back(buffer[0]);
    }
  }

  void bezier_curve(const std::vector<std::complex<double>> &in, int in_index, int blocks, std::vector<std::complex<double>> &output){
    int point_c = blocks - 1;
    for(int i = 0; i<blocks; i++){
      bezier_point(in, in_index, (double)i/(double)point_c, output);
    }
  }

  void build_smooth_path(const path_t &input, std::vector<std::complex<double>> &output){
    std::vector<std::complex<double>> tangents;
    find_tangents(input.nodes, tangents);
    for(int i = 0; i < (tangents.size() / 4); i++){
      bezier_curve(tangents, i * 4, 20, output);
    }
  }

  void test_build_smooth_path(void){
    std::vector<node_t> g;
    g.push_back({0,0,0,0,0  ,0,0,GenKind::victim,{},0});
    g.push_back({0,0,0,0,51 ,1,0,GenKind::victim,{},0});
    g.push_back({0,0,0,0,200,4,0,GenKind::victim,{},0});
    g.push_back({0,0,0,0,120,5,0,GenKind::victim,{},0});
    path_t p;
    p.nodes.push_back(&g[0]);
    p.nodes.push_back(&g[1]);
    p.nodes.push_back(&g[2]);
    p.nodes.push_back(&g[3]);
    std::vector<std::complex<double>> u;
    build_smooth_path(p, u);
    for(auto p : u){
      std::cout << real(p) << ":" << imag(p) << std::endl;
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
  test_build_smooth_path();
  return 0;
  vector<area_t> a;
  for(double x = 0; x < 20; x++){
      a.push_back({GenKind::victim, x, x, x + 1.0, x + 1.0, -1});
    
  }
  /*a.push_back({GenKind::obstacle, 0, 0, 1, 1, 1});
  a.push_back({GenKind::victim, 1, 1, 2, 2, 1});
  */
  a.push_back({GenKind::target, 0, 20, 1, 21, -1});
  a.push_back({GenKind::start, 1, 0, 2, 1, -1});
  vector<node_t> t;
  vector<node_t*> starts;
  vector<node_t*> ends;
  path_planner::graph_from_poly(a, t, starts, ends);
  path_t res;
  plan(t, starts[0], res, 20, 1);
  vector<std::complex<double>> out; 
  build_smooth_path(res, out);
  for(auto p : out){
    cout << real(p) << " : " << imag(p) << endl;
  }
  return 0;
}
#endif