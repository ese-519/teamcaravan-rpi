from PIL import Image
from PIL import ImageDraw
import os.path
import math

def rectangle_contains(p1, p2, p_test):
    in_x = (p1[0] <= p_test[0] and p_test[0] <= p2[0]) or (p1[0] >= p_test[0] and p_test[0] >= p2[0])
    in_y = (p1[1] <= p_test[1] and p_test[1] <= p2[1]) or (p1[1] >= p_test[1] and p_test[1] >= p2[1])
    
    return in_x and in_y 

class hallway:
    def __init__(self, s_id, length, ang = 0, e_n=[], e_p=[]):
        self.s_id = s_id
        self.length = length
        self.angle = ang
        self.edges_next = []
        self.start_coords = [0, 0]
        self.end_coords = [0, 0]
        for e in e_n:
          self.edges_next.append(e)
          
        self.edges_prev = []
        for e in e_p:
          self.edges_prev.append(e)

    def setStartCoords(self, c):
      self.start_coords[0] = int(c[0])
      self.start_coords[1] = int(c[1])
      
      self.end_coords[0] = int(self.start_coords[0] + self.length * math.cos(math.radians(self.angle)))
      self.end_coords[1] = int(self.start_coords[1] + self.length * math.sin(math.radians(self.angle)))
      
    def __str__(self):
      res = ""
      for e in self.edges_next:
        res += "\tN\t" + str(e[0]) + "\t" + str(e[1]) + "\n"
      for e in self.edges_prev:
        res += "\tP\t" + str(e[0]) + "\t" + str(e[1]) + "\n"
      res += str(self.s_id) + "\t" + str(self.length) + "\t" + str(self.angle) + "\n"
      return res
        
class Map:

    def __str__(self):
        res = ""
        for h in self.hallways:
          res += str(h)
        return res

    def __init__(self, path):
        self.hallways = []
        self.start = None
        self.curId = None
        self.prog = 0
        self.forward = True
        self.readMap(path)
        
    def addStart(self, start, off=0):
        self.start = start
        self.start.setStartCoords([0, 0])
        self.curId = start.s_id
        self.prog = off/start.length
        self.forward = True
        self.hallways.append(self.start)
    
    def addHallway(self, hlwy):
        for h in self.hallways:
            if (h.s_id == hlwy.s_id):
              return
        self.hallways.append(hlwy)
        
    def setStartCoords(self, c, i=1):
        for h in self.hallways:
          if h.s_id == i:
            h.setStartCoords(c)   
            
    def getHallway(self, i):
        for h in self.hallways:
            if h.s_id == i:
                return h
        return None 
    
    def readMap(self, path): 
      if (not os.path.isfile(path)):
        return False
      with open(path) as f:
        content = f.readlines()
        e_n = []
        e_p = []
        for l in content:
          hlwy = l.split()
          if (len(hlwy) < 3):
              return False
            
          if (hlwy[0] == 'N'):
            e_n.append([int(hlwy[1]), int(hlwy[2])])
          elif (hlwy[0] == 'P'):
            e_p.append([int(hlwy[1]), int(hlwy[2])])
          else:
            h = hallway(int(hlwy[0]), int(hlwy[1]), int(hlwy[2]), e_n, e_p)
            if (self.start == None):
              self.addStart(h)
            else:
              self.addHallway(h)
            e_n = []
            e_p = []
            
    def hallwayLen(self, id):
        for h in self.hallways:
          if h.s_id == id:
            return h.length
        return -1
    
    def getBound(self):
      b1 = [0, 0]
      b2 = [self.start.end_coords[0], self.start.end_coords[1]]
      
      for h in self.hallways:
        for e in h.edges_next:
            self.setStartCoords(h.end_coords, e[1])
            e_h = self.getHallway(e[1])            
            n_p = [e_h.end_coords[0], e_h.end_coords[1]]
            
            if not rectangle_contains(b1, b2, n_p):
              #update bound
              b1[0] = min(b1[0], b2[0], n_p[0])
              b2[0] = max(b1[0], b2[0], n_p[0])
              b1[1] = min(b1[1], b2[1], n_p[1])
              b2[1] = max(b1[1], b2[1], n_p[1]) 
      
      return [int(b1[0]), int(b1[1]), int(b2[0]), int(b2[1])]
    
    def createImage(self, path):
      scale = 30
      margin = 25
      b = self.getBound()
      s = max(abs(b[0]-b[2]), abs(b[1]-b[3])) * scale + 2 * margin
      size = s, s 
      im = Image.new("1", size, 1)
      
      draw = ImageDraw.Draw(im)
      
      for h in self.hallways:
        xy = [margin + scale * h.start_coords[0], s - (margin + scale * h.start_coords[1]), margin + scale * h.end_coords[0], s - (margin + scale * h.end_coords[1])]
        draw.line(xy, 0, 5) 
      
      im.save(path, "JPEG")
      
         
    
#m = Map("levine.mp")
#m.createImage("levine.jpg")