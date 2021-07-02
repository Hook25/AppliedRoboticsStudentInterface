import heapq
from copy import copy

PRIZE = 5

class ExplorerPQ:
  def __init__(self, cost_function):
    self.cost_function = cost_function
    self._inner = []
  def push(self, item):
    heapq.heappush(self._inner, (*self.cost_function(item), item))
  def pop(self):
    return heapq.heappop(self._inner)[-1]
  def empty(self):
    return len(self._inner) == 0

class Path:
  def __init__(self):
    self.cost = 0
    self.nodes = []
    self.visited_prize = {}
  def new_append(self, node):
    p = Path()
    p.cost = self.cost
    p.nodes = copy(self.nodes)
    p.visited_prize = copy(self.visited_prize)
    p.append(node)
    return p
  def append(self, node):
    self.nodes.append(node)
    self.cost += self.last_step_cost()
  def last_step_cost(self):
    if len(self.nodes) == 1:
      return 0
    base = ( #TODO: simple for now
      abs(self.nodes[-1].x - self.nodes[-2].x) +
      abs(self.nodes[-1].y - self.nodes[-2].y)
    )
    if (self.nodes[-1].kind == self.nodes[-1].VICTIM and
      self.nodes[-1].uvic_id not in self.visited_prize):
      self.visited_prize[self.nodes[-1].uvic_id] = True
      base -= PRIZE
    return base
  def __getitem__(self, i):
    return self.nodes[i]
  def __iter__(self):
    return iter(self.nodes)
  def __lt__(self, other):
    return id(self) < id(other)

def clean(g):
  for row in g.rows:
    for n in row:
      n.cost = 2e9
      n.visited = False

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
        res = new_path
      elif new_path.cost < nei.cost and (
       res == None or res.cost > new_path.cost):
        nei.cost = new_path.cost
        epq.push(new_path)
      """else:
        if new_path.cost >= nei.cost:
          print("discarded:", new_path.cost, "worse than: ", nei.cost)
        else:
          print("discarded:", new_path.cost, "worse than: ", res.cost)"""
  return res

def init_dist(g, target):
  to_visit = [target]
  while to_visit:
    now = to_visit.pop()
    if not now.visited:
      now.dst_target = (abs(now.x - target.x) + abs(now.y - target.y))
      now.visited = True
      for nei in now.nei:
        to_visit.append(nei)

def cf_dst(path):
  return path[-1].dst_target

def cf_cost(path):
  return path.cost

def cf(path):
  dst, cost = cf_dst(path), cf_cost(path)
  return (dst + cost, cost)

def main():
  from data import Area, Node, Grid
  from utils import draw_graph, draw_path
  xs = list(range(90))
  ys = list(range(90))
  areas = [Area([(x, y), (x+1,y+1)], Node.VICTIM) for (x,y) in zip(xs, ys)]
  g = Grid()
  g.from_poly(areas)
  graph = g.to_graph()
  target = graph.rows[-1][-1]
  target.kind = target.TARGET

  clean(graph)
  init_dist(graph, target)

  from time import time
  print("started")
  t = time()
  path = find_path(graph.rows[0][0], target, cf)
  print((time() - t), "s")
  draw_path(path)
  draw_graph(graph)

if __name__ == "__main__":
  main()
