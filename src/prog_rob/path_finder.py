import heapq
from copy import copy, deepcopy

PRIZE = 2

class ExplorerPQ:
  def __init__(self, cost_function):
    self.cost_function = cost_function
    self._inner = []
  def push(self, item):
    heapq.heappush(self._inner, (self.cost_function(item), item))
  def pop(self):
    return heapq.heappop(self._inner)[-1]
  def empty(self):
    return len(self._inner) == 0

class Path:
  def __init__(self):
    self.cost = 0
    self.nodes = []
    self.visit_dict = {}
  def new_append(self, node):
    p = Path()
    p.cost = self.cost
    p.nodes = copy(self.nodes)
    p.visit_dict = copy(self.visit_dict)
    p.append(node)
    return p
  def append(self, node):
    self.nodes.append(node)
    self.cost += self.last_step_cost()
    self.visit_dict[node] = True
  def last_step_cost(self):
    if len(self.nodes) == 1:
      return 0
    base = ( #TODO: simple for now
      abs(self.nodes[-1].x - self.nodes[-2].x) +
      abs(self.nodes[-1].y - self.nodes[-2].y)
    )
    if (self.nodes[-1].kind == self.nodes[-1].VICTIM and
      self.nodes[-1] not in self.visit_dict):
      base -= PRIZE
    return base
  def __getitem__(self, i):
    return self.nodes[i]
  def __iter__(self):
    return iter(self.nodes)
  def __lt__(self, other):
    return self.cost < other.cost

def clean(g):
  for row in g.rows:
    for n in row:
      n.cost = 2e9

def find_path(start, target, cost_function):
  start_path = Path()
  start_path.append(start)
  epq = ExplorerPQ(cost_function)
  epq.push(start_path)
  res = None
  while not epq.empty():
    p_now = epq.pop()
    for nei in p_now[-1].nei:
      new_path = p_now.new_append(nei)
      if nei.kind == nei.TARGET:
        nei.cost = new_path.cost
        res = new_path
      elif new_path.cost < nei.cost and (
       res == None or res.cost > new_path.cost
      ):
        nei.cost = new_path.cost
        epq.push(new_path)
      """else:
        if new_path.cost >= nei.cost:
          print("discarded:", new_path.cost, "worse than: ", nei.cost)
        else:
          print("discarded:", new_path.cost, "worse than: ", res.cost)"""
  return res

def main():
  from data import Area, Node, Grid
  from utils import draw_graph, draw_path, draw_graph_3d
  xs = list(range(150))
  ys = list(range(150))
  areas = [Area([(x, y), (x+1,y+1)], Node.VICTIM) for (x,y) in zip(xs, ys)]
  g = Grid()
  g.from_poly(areas)
  graph = g.to_graph()
  target = graph.rows[-1][-1]
  graph.rows[1][1].kind = target.OBSTACLE
  graph.rows[2][1].kind = target.OBSTACLE
  graph.rows[1][2].kind = target.OBSTACLE
  graph.rows[2][2].kind = target.OBSTACLE
  target.kind = target.TARGET

  def cf(path):
    return (abs(target.x - path[-1].x) + abs(target.y - path[-1].y))

  clean(graph)
  from time import time
  print("started")
  t = time()
  path = find_path(graph.rows[0][0], target, cf)
  print((time() - t), "s")
  draw_path(path)
  draw_graph(graph)
  draw_graph_3d(graph)

if __name__ == "__main__":
  main()
