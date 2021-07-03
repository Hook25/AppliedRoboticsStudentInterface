import cv2
import numpy as np
from math import sqrt, atan2
from prog_rob import path_finder

GREEN_LOW = np.array([54,127,0], dtype="uint8")
GREEN_HIGH = np.array([98, 255, 255], dtype="uint8")

RED_LOW = np.array([0,171,0], dtype="uint8")
RED_HIGH = np.array([43,255,255], dtype="uint8")

BLUE_LOW = np.array([99, 180, 0], dtype="uint8")
BLUE_HIGH = np.array([123, 240, 255], dtype="uint8")

template_name =  ["1.jpg", "3.jpg", "4.jpg", "5.jpg"]
templates = [cv2.imread(path) for path in template_name]

#img in BGR2HSV
def find_col(img, crange):
  mask = cv2.inRange(img, crange[0], crange[1])
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4,4))
  #opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
  opening = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
  #opening = cv2.dilate(opening, kernel,iterations = 1)
  cnts, _ = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  return cnts

def points_dst(p1, p2):
  return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1])**2)

def approx_cont(cnt, param):
  epsilon = param*cv2.arcLength(cnt,True)
  return cv2.approxPolyDP(cnt,epsilon,True)

def get_gate(conts):
  conts_edges = [(cont, len(cont), i) for i, cont in enumerate(conts)]
  m = min(s for _, s, _ in conts_edges)
  found, index = [(x, i) for x, s, i in conts_edges if s == m][0]
  gate = cv2.boxPoints(cv2.minAreaRect(found))
  del conts[index]
  return np.int0(gate)

def _get_victim(img, x, y, rad):
  global templates
  loc = img.copy()
  w,h, *_ = img.shape
  mask = np.zeros((w, h), np.uint8)
  cv2.circle(mask, (int(x), int(y)), int(rad), (255,), -1)
  masked_data = cv2.bitwise_and(loc, loc, mask=mask)
  masked_data = cv2.cvtColor(masked_data, cv2.COLOR_HSV2BGR)
  return template_matching(masked_data, templates)
  
def get_victims(img, conts):
  x_y_rads = [cv2.minEnclosingCircle(cnt) for cnt in conts]
  return [(x, y, rad, _get_victim(img, x, y, rad)) for (x, y), rad in x_y_rads]
  
def find_victims_gate(img, crange):
  conts = find_col(img, crange)
  conts_size = [(cnt, cv2.contourArea(cnt)) for cnt in conts]
  mx = max([s for c,s in conts_size])
  conts = [x for x, s in conts_size if s > (float(mx)* .3)]
  conts = [approx_cont(x, .01) for x in conts]
  gate = get_gate(conts)
  victims = get_victims(img, conts)
  return (victims, gate)

def find_robot(img, crange):
  conts = find_col(img, crange)
  conts_size = [(cnt, cv2.contourArea(cnt)) for cnt in conts]
  mx = max([s for c,s in conts_size])
  bot = [c for c,s in conts_size if s == mx][0]
  _, bot = cv2.minEnclosingTriangle(bot)
  bari_p = [
    bot[0][0][0]/3 + bot[1][0][0]/3 + bot[2][0][0]/3, 
    bot[0][0][1]/3 + bot[1][0][1]/3 + bot[2][0][1]/3
  ]
  
  top = max(points_dst(bari_p, point[0]) for point in bot)
  top_p = [point[0] for point in bot if points_dst(bari_p, point[0]) == top][0]
  bari_p = tuple([int(z) for z in bari_p])
  top_p = tuple([int(z) for z in top_p])
  theta = atan2(bari_p[1] - top_p[1], bari_p[0] - top_p[0]) 
  return np.int0(bot), bari_p, top_p, theta

def filter_by_size(cont, m):
  c_size = [(c, cv2.contourArea(c)) for c in cont]
  M_size = max(y for x,y in c_size)
  return [c for (c, size) in c_size if size > M_size * m]

def find_obstacles(img, crange):
  conts = find_col(img, crange)
  conts = [approx_cont(c, .03) for c in conts]
  conts = filter_by_size(conts, m=.01)
  return conts
 
def template_matching(img, templates):
  methods = [cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF_NORMED] 
  best_cmp_val = 0
  best_cmp_i = 0
  img2 = img.copy()
  for i, template in enumerate(templates):
    w, h, *_ = template.shape
    for method in methods:
      res = cv2.matchTemplate(img,template,method)
      min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
      cmp_val = max_val
      if method == cv2.TM_SQDIFF_NORMED:
        cmp_val = (1-min_val)
      if cmp_val > best_cmp_val:
        best_cmp_val = cmp_val
        best_cmp_i = i
      img = img2.copy()
  return best_cmp_i
        
def cont_p_to_sensible(cont):
  return [p[0] for p in cont] # p = [[x,y]] -> [x,y]

def circle_to_box(c):
  cx, cy, r, i = c
  x, y, Mx, My = cx - r, cy - r, cx + r, cy + r
  return [(x,y), (Mx, My)], i

MARGIN = .1
def enlarge_obst(obst):
  scale = MARGIN / 2
  dec = 1 - scale
  inc = 1 + scale
  obst.x *= dec
  obst.y *= dec
  obst.Mx *= inc
  obst.My *= inc
  return obst

def enlarge_obsts(obsts):
  return [enlarge_obst(o) for o in obsts]

def do_path_finding(rob, victs, obsts, gate):
  from prog_rob import data
  from prog_rob.utils import center_of, draw_graph, draw_path
  from prog_rob import path_finder
  rob = cont_p_to_sensible(rob)
  obsts = [cont_p_to_sensible(obst) for obst in obsts]
  victs = [circle_to_box(c) for c in victs]
  
  area_obst = [data.Area(obst, data.Node.OBSTACLE) for obst in obsts]
  area_obst = enlarge_obsts(area_obst)

  areas = [data.Area(vict[0], data.Node.VICTIM, vict[1]) for vict in victs]
  areas += area_obst
  areas += [data.Area(rob, data.Node.START)]
  target = data.Area(gate, data.Node.TARGET)
  areas += [target]
  g = data.Grid()
  g.from_poly(areas)
  graph = g.to_graph()
  start = None
  for row in graph.rows:
    for node in row:
      if node.kind == node.TARGET:
        target = node
  for row in graph.rows:
    for node in row:
      if node.kind == node.START:
        start = node
  path_finder.clean(graph)
  path_finder.init_dist(graph, target)
  poly_path = path_finder.find_path(start, target, path_finder.cf)
  poly_coords_path = [complex(*center_of((n.x, n.Mx), (n.y, n.My))) for n in poly_path]
  draw_graph(graph)
  draw_path(poly_path)
  return poly_coords_path

def do_smooth_path(poly):
  from curve_maker import build_smooth_path, plot_compl
  import matplotlib.pyplot as plt 
  smooth = build_smooth_path(poly)
  plot_compl(smooth, plt.plot)
  plt.show()

def main():
  img = cv2.imread("in.jpg")
  orig = img.copy()
  img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  cont, bari_p, top_p, angle = find_robot(img, (BLUE_LOW, BLUE_HIGH))
  victs, gate = find_victims_gate(img, (GREEN_LOW, GREEN_HIGH))
  obst = find_obstacles(img, (RED_LOW, RED_HIGH))
  poly = do_path_finding(cont, victs, obst, gate)
  do_smooth_path(poly)
  return
  cv2.drawContours(orig, [cont], 0, (0,0,0), 2)
  cv2.circle(orig, bari_p, 5,(0,0,255), -1)
  cv2.circle(orig, top_p, 5,(0,255,0), -1)
  cv2.drawContours(orig, [gate], 0, (0,0,0), 2)
  for (x, y, _, txt) in victs:
    cv2.putText(orig, template_name[txt],(int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
  print(len(obst))
  cv2.drawContours(orig, obst, -1, (255,0,0), 3)
  cv2.imshow("Done", orig)
  cv2.waitKey()
          
if __name__ == "__main__":
  main()
