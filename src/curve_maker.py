import matplotlib.pyplot as plt 

def bezier_curve(control_points, number_of_curve_points):
  last_point = number_of_curve_points - 1
  return [ bezier_point(control_points, i / last_point )
    for i in range(number_of_curve_points)
  ]

def bezier_point(control_points, t):
  while len(control_points) > 1:
    control_linestring = zip(control_points[:-1], control_points[1:])
    control_points = [(1 - t) * p1 + t * p2 for p1, p2 in control_linestring]
  return control_points[0]
  
def plot_compl(compl_arr, df, **kwargs):
  x = [point.real for point in compl_arr]
  y = [point.imag for point in compl_arr]
  df(x,y, **kwargs)

a = 1

def d(p, i):
  return abs(p[i] - p[i-1]) ** a  

def calc_intermediate(p):
  assert len(p) == 4, "Error"
  d1 = d(p, 1)
  d2 = d(p, 2)
  d3 = d(p, 3)
  d1_sq = d1**2
  d2_sq = d2**2
  d3_sq = d3**2
  t1 = (d1_sq * p[2] - d2_sq * p[0] + (2*d1_sq + 3 * d1 * d2 + d2_sq) * p[1]) / (3 * d1 * (d1 + d2))
  t2 = (d3_sq * p[1] - d2_sq * p[3] + (2*d3_sq + 3 * d3 * d2 + d2_sq) * p[2]) / (3 * d3 * (d3 + d2))
  return [p[1], t1, t2, p[2]]

def find_tangents(p):
  p = [p[0] + .001*(p[0] - p[1])] + p + [p[-1] + .001*(p[-1] - p[-2])] #append fake start and end
  res = []
  for i in range(int(len(p) - 4 + 1)):
    res += calc_intermediate(p[i:i+4])
  return res
  
def build_smooth_path(path):
  tangents = find_tangents(path)
  points = []
  for i in range(int(len(tangents)/4)):
    points += bezier_curve(tangents[i*4: (i*4)+4], 10)
  return points
  
def main():
  arr = [(0,0), (50, 1.5), (200, 4.5), (230, 4.5)]
  arr = [complex(x,y) for (x,y) in arr]
  points = build_smooth_path(arr)
  plot_compl(points, plt.plot, c='b')
  plot_compl([complex(230, 4.5), complex(1000, 4.5)], plt.plot, c='b')
  #plot_compl(arr_to_draw, plt.scatter, c='g')
  plot_compl(arr, plt.scatter, c='r')
  plt.show()
  
  
if __name__ == '__main__':
  main()