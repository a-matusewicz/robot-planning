from ArmRobot import ArmRobot
import astar_search
from shapely.geometry import Polygon, Point
import math


# p1 = Polygon([(100, 100), (100, 125), (125, 125), (125, 100)])
# p2 = Polygon([(-100, 100), (-125, 100), (-125, 125), (-100, 125)])
# p3 = Polygon([(-100, -100), (-100, -125), (-125, -125), (-125, -100)])
# p4 = Polygon([(100, -100), (125, -100), (125, -125), (100, -125)])
#
# start_angles = [math.pi / 2, -math.pi / 3, math.pi / 2]
# goal_angles = [-math.pi / 2, math.pi / 3, -math.pi / 2]
#
# rob = ArmRobot([100, 100, 50], start_angles, goal_angles, [p1, p2, p3, p4])
# # print(rob.start_locations)
# # print(rob.collides(rob.start_angles))
# # print(len(rob.get_vertices(10)))
# print(rob.build_roadmap(2, 5))
# print(rob.random_points)
# DrawModel(500, 500, rob)
# d.draw()


p1 = Polygon([(100, 100), (100, 125), (125, 125), (125, 100)])
p2 = Polygon([(-100, 100), (-125, 100), (-125, 125), (-100, 125)])
p3 = Polygon([(-100, -100), (-100, -125), (-125, -125), (-125, -100)])
p4 = Polygon([(100, -100), (125, -100), (125, -125), (100, -125)])

# start_angles = [math.pi / 2, -math.pi / 3, math.pi / 2]
# goal_angles = [-math.pi / 2, math.pi / 3, -math.pi / 2]
# arm_lengths = [100, 100, 50]

start_angles = (math.pi / 2, 5 * math.pi / 3)
goal_angles = (3 * math.pi / 2, math.pi / 3)
arm_lengths = [100, 100]

arm = ArmRobot(arm_lengths, start_angles, goal_angles, [p1, p2, p3, p4], 25, 5)
height = 500
width = 500

# road_map = arm.build_roadmap(2, 5)
# print(arm.road_map)
# print(arm.random_points)
# print(1.5%6)
# print(goal_angles)
# print(arm.random_points[5])
# print(arm.angle_distance(5))

# print(astar_search.astar_search(arm, arm.angle_distance))
angle1 = (3 * math.pi / 2, math.pi / 3)
angle2 = (3 * math.pi / 2 - math.pi, math.pi / 3)
dist = 0
for i in range(len(angle1)):
    dist += abs(angle1[i] - angle2[i]) % (2 * math.pi)

print(dist)
