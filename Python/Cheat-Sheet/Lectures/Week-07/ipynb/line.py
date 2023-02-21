class myLine: 
#constructor
    def __init__(self,a=0,b=0,c=1,d=1):
        self.x1=a; self.y1=b
        self.x2=c; self.y2=d
    
    def length(self): #returns length 
        import math
        dx = self.x1-self.x2 
        dy = self.y1-self.y2
        ll = math.sqrt(dx*dx + dy*dy)
        return ll
    
    def moveX(self, dx1=0,dx2=0):
        self.x1 += dx1
        self.x2 += dx2

    def moveY(self, dy1=0,dy2=0):
        self.y1 += dy1
        self.y2 += dy2
    
    def slope(self):
        return (self.y2-self.y1)/(self.x2-self.x1)