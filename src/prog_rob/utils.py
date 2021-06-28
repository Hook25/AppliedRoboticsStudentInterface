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
  plt.plot([n.x, n.x, n.Mx, n.Mx, n.x], [n.y, n.My, n.My, n.y, n.y], c=c)

def draw_graph(g):
  for row in g.rows:
    for node in row:
      draw_node(node)
  draw_connectins(g)
  plt.show()

def draw_connectins(g):
  for row in g.rows:
    for node in row:
      for nei in node.nei:
        c_node = center_of([node.x, node.Mx], [node.y, node.My])
        c_nei = center_of([nei.x, nei.Mx], [nei.y, nei.My])
        plt.plot([c_node[0], c_nei[0]], [c_node[1], c_nei[1]])

def main():
  from data import Grid
  g = Grid()
  g.xs = list(range(30))
  g.ys = list(range(30))
  draw_grid(g)
  
if __name__ == "__main__":
  main()