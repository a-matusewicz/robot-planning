# using shapely for collision detection

from shapely.geometry import Polygon, Point
from MobileRobot import MobileRobot

# poly = Polygon(((0, 0), (0, 1), (1, 1), (1, 0)))
# point = Point(2, .2)
#
# print(poly)
# print(poly.contains(point))

p1 = Polygon([(100, 100), (100, 125), (125, 125), (125, 100)])
p2 = Polygon([(-100, 100), (-125, 100), (-125, 125), (-100, 125)])
p3 = Polygon([(-100, -100), (-100, -125), (-125, -125), (-125, -100)])
p4 = Polygon([(100, -100), (125, -100), (125, -125), (100, -125)])

rob = MobileRobot((100, 100, 0), (150, 150), [p1, p2, p3, p4])

rob.rrt(1)
