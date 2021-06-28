
def clean(g):
  for row in g.rows:
    for n in row:
      n.cost = 0

def find_path(start, targets):
  loc = start
  
  while targets:
    target =  targets.pop()
    