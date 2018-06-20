import datetime, math

def getTimestamp(dt):
  return (dt - datetime.datetime(1970, 1, 1)).total_seconds()

def dateRange(d1, d2, step):
  curr = d1
  while curr < d2:
    yield curr
    curr += step

def euclideanDist(p1, p2):
  #You know what this does
  res = 0
  for pair in zip(p1, p2):
    res += (pair[1] - pair[0])**2

  return math.sqrt(res)
