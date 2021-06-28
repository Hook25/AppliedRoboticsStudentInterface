from matplotlib import pyplot as plt

def draw_grid(grid):
  max_x = max(grid.xs)
  max_y = max(grid.ys)
  for x in grid.xs: 
    plt.plot([x, x], [0, max_y], color='black')
  for y in grid.ys:
    plt.plot([0, max_x], [y, y], color='black')
  plt.show()

def avg(s):
  return sum(s) / len(s)

def center_of(xs, ys):
  return avg(xs), avg(ys)

def draw_node(n):
  c = "black"
  if n.kind == n.OBSTACLE:
    c = "red"
  elif n.kind == n.VICTIM:
    c = "green"
  elif n.kind == n.TARGET:
    c = "blue"
  elif n.kind == n.FREE:
    return
  plt.plot([n.x, n.x, n.Mx, n.Mx, n.x], [n.y, n.My, n.My, n.y, n.y], c=c)

def draw_graph(g):
  for row in g.rows:
    for node in row:
      draw_node(node)
  plt.show()

def draw_connectins(g):
  for row in g.rows:
    for node in row:
      for nei in node.nei:
        c_node = center_of([node.x, node.Mx], [node.y, node.My])
        c_nei = center_of([nei.x, nei.Mx], [nei.y, nei.My])
        plt.plot([c_node[0], c_nei[0]], [c_node[1], c_nei[1]])

def draw_path(p):
  centers = [(center_of([n.x, n.Mx], [n.y, n.My])) for n in p]
  plt.plot([n[0] for n in centers], [n[1] for n in centers], c="blue")

def draw_graph_3d(g):
  X = list(range(50))
  Y = list(range(50))
  Z = [[node.cost if node.cost < 100 else 0 for node in row] for row in  g.rows]
  fig = plt.figure()
  ax = plt.axes(projection='3d')
  ax.contour3D(X, Y, Z, 50, cmap='binary')
  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_zlabel('value');
  fig.show()
  plt.show()

def main():
  from data import Grid
  g = Grid()
  g.xs = list(range(30))
  g.ys = list(range(30))
  draw_grid(g)
  
if __name__ == "__main__":
  main()
