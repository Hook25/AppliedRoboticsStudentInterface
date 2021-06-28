import heapq
from copy import copy

class ExplorerPQ:
  def __init__(self, cost_function):
    self.cost_function = cost_function
    self._inner = []
    self._selector = 0
  def push(self, item):
    heapq.heappush(self._inner, (self.cost_function(item), self._selector, item))
    self._selector += 1
  def pop(self):
    return heapq.heappop(self._inner)[-1]
  def empty(self):
    return len(self._inner) == 0

class Path:
  def __init__(self):
    self.cost = 0
    self.nodes = []
  def new_append(self, node):
    p = Path()
    p.cost = self.cost
    p.nodes = copy(self.nodes)
    p.append(node)
    return p
  def append(self, node):
    self.nodes.append(node)
    self.cost += self.last_step_cost()
  def last_step_cost(self):
    if len(self.nodes) == 1:
      return 0
    return ( #TODO: simple for now
      abs(self.nodes[-1].x - self.nodes[-2].x) +
      abs(self.nodes[-1].y - self.nodes[-2].y)
    )
  def __getitem__(self, i):
    return self.nodes[i]
  def __iter__(self):
    return iter(self.nodes)

def clean(g):
  for row in g.rows:
    for n in row:
      n.cost = 2e9

def find_path(start, target, cost_function):
  done_paths = []
  start_path = Path()
  start_path.append(start)
  epq = ExplorerPQ(cost_function)
  epq.push(start_path)
  while not epq.empty():
    p_now = epq.pop()
    for nei in p_now[-1].nei:
      new_path = p_now.new_append(nei)
      if nei.kind == nei.TARGET:
        done_paths.append(new_path)
      elif new_path.cost < nei.cost:
        nei.cost = new_path.cost
        epq.push(new_path)
  return done_paths

def main():
  from data import Area, Node, Grid
  from utils import draw_graph, draw_path
  xs = list(range(20))
  ys = list(range(20))
  areas = [Area([(x, y), (x+1,y+1)], Node.FREE) for (x,y) in zip(xs, ys)]
  g = Grid()
  g.from_poly(areas)
  graph = g.to_graph()
  target = graph.rows[-1][-1]
  target.kind = target.TARGET
  def cf(path):
    return (abs(target.x - path[-1].x) + abs(target.y - path[-1].y))
  clean(graph)
  paths = find_path(graph.rows[0][0], target, cf)
  for path in paths:
    draw_path(path)
  draw_graph(graph)

if __name__ == "__main__":
  main()
