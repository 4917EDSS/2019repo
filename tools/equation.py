from __future__ import division
def calculate(values) :
    x1,x2,y1,y2=values[0][0],values[1][0],values[0][1],values[1][1]
    m=float((y2-y1)/(x2-x1))
    b=y1-(m*x1)
    return [m,b]