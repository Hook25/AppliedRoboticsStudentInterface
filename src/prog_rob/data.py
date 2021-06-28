X = 0
Y = 1

class Grid:
  def __init__(self):
    self.xs = []
    self.ys = []
    self.areas = []
  def from_poly(self, areas):
    self.areas = areas
    for area in areas:
      mx, Mx, my, My = area.to_box()
      self.xs += [mx, Mx]
      self.ys += [my, My]
    self.xs = list(set(self.xs))
    self.ys = list(set(self.ys))
  
  def categorize(self, node):
    inter = [a for a in self.areas if node.box_intersects(a)]
    return inter[0].kind if len(inter) > 0 else Node.FREE
  
  def to_graph(self):
    g = Graph()
    if 0 not in self.xs:
      self.xs = [0] + self.xs
    if 0 not in self.ys:
      self.ys = [0] + self.ys
    loc_xs = self.xs[:-1]
    loc_ys = self.ys[:-1]
    for x_i, x in enumerate(loc_xs):
      g.rows.append([
        Node(x, self.xs[x_i+1], y, self.ys[y_i+1]) 
        for y_i, y in enumerate(loc_ys)
      ])
    r_c = len(g.rows)
    n_c = len(g.rows[0])
    for r_i, row in enumerate(g.rows):
      for n_i, node in enumerate(row):
        #g.rows[r_i][n_i]
        node.kind = self.categorize(node)
        if r_i > 0:
          node.nei.append(g.rows[r_i-1][n_i])
        if n_i > 0:
          node.nei.append(g.rows[r_i][n_i-1])
        if r_i < r_c - 1:
          node.nei.append(g.rows[r_i + 1][n_i])
        if n_i < n_c -1:
          node.nei.append(g.rows[r_i][n_i + 1])
    return g

class Area:
  def __init__(self, poly, kind):
    self.poly = poly
    self.kind = kind
  def to_box(self):
    mx, Mx = min(p[X] for p in self.poly), max(p[X] for p in self.poly)
    my, My = min(p[Y] for p in self.poly), max(p[Y] for p in self.poly)
    return mx, Mx, my, My
  def corner_inside(self, other):
    o_mx, o_Mx, o_my, o_My = other.to_box()
    mx, Mx, my, My = self.to_box()
    return (mx >= o_mx and mx <= o_Mx and my >= o_my and my <= o_My)
  def box_intersects(self, other):
    return (
      self.corner_inside(other) or
      (other.corner_inside(self))  #He's inside me
    )

class Graph:
  def __init__(self):
    self.rows = []
 
class Node(Area):
  FREE = 0
  OBSTACLE = 1
  VICTIM = 2
  UNCAT = 3
  def __init__(self, x, Mx, y, My):
    x, Mx = min(x, Mx), max(x, Mx)
    y, My = min(y, My), max(y, My)
    self.x = x
    self.Mx = Mx
    self.y = y
    self.My = My
    self.nei = []
    super().__init__([(x, y), (Mx, My)], Node.UNCAT)

def main():
  from utils import  draw_graph
  xs = list(range(20))
  ys = list(range(20))
  areas = [Area([(x, y), (x+1,y+1)], Node.FREE) for (x,y) in zip(xs, ys)]
  g = Grid()
  g.from_poly(areas)
  graph = g.to_graph()
  draw_graph(graph)

if __name__ == "__main__":
   main()