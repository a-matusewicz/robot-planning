"""
Anna Matusewicz
CS 76 Prof. Balkcom
11/16/17
"""

from cs1lib import *
from ArmRobot import ArmRobot
import astar_search
from shapely.geometry import Polygon, Point
import math
from time import sleep

height = 500
width = 500


# Draws arm and obstacles in starting position (Only used for testing)
def draw():
    global arm, height, width

    org_x = x1 = width / 2
    org_y = y1 = height / 2

    if arm.collides(arm.random_points[0]):
        set_clear_color(1, 0, 0)
        clear()

    set_stroke_color(0, 0, 0)
    set_fill_color(0, 0, 0)
    for ob in arm.obstacles:
        verts = []
        for coord in ob.exterior.coords:
            verts.append((org_x + coord[0], org_y - coord[1]))
        draw_polygon(verts)

    set_stroke_color(0, .5, 0)
    for link in arm.compute_locations(arm.random_points[0]):
        draw_line(x1, y1, org_x + link[0], org_y - link[1])

        x1 = org_x + link[0]
        y1 = org_y - link[1]


# Draws the configuration space
def draw_config():
    global arm, height, width, road_map
    org_x = width / 2
    org_y = height / 2

    clear()

    # Draws obstacles
    set_stroke_color(0, 0, 0)
    set_fill_color(0, 0, 0)
    for ob in arm.obstacles:
        verts = []
        for coord in ob.exterior.coords:
            verts.append((org_x + coord[0], org_y - coord[1]))
        draw_polygon(verts)

    set_stroke_color(0, .5, 0)

    # Draws potential arm endpoints
    for potential in road_map:
        for angle in road_map[potential]:
            start = arm.compute_locations(arm.random_points[potential])
            chain = arm.compute_locations(arm.random_points[angle])

            set_stroke_color(0, .5, 0)
            draw_line(org_x + start[1][0], org_y - start[1][1], org_x + chain[1][0], org_y - chain[1][1])

            if potential == 0 or potential == 1:
                set_fill_color(1, 0, 0)
                set_stroke_color(1, 0, 0)
                draw_ellipse(org_x + start[1][0] - 1, org_y - start[1][1] - 1, 5, 5)
            else:
                set_stroke_color(0, 0, 0)
                set_fill_color(0, 0, 0)
                draw_ellipse(org_x + start[1][0] - 1, org_y - start[1][1] - 1, 2, 2)


# Animates the solution
def draw_answer():
    global arm, height, width, answer, position
    org_x = width / 2
    org_y = height / 2

    # Detects if collision took place
    if arm.collision:
        set_clear_color(1, 0, 0)

    # clear()

    set_stroke_color(0, .5, 0)

    step()
    sleep(.25)

    # for position in answer.path:
    x1 = org_x
    y1 = org_y
    set_stroke_color(0, .5, 0)
    pos = arm.compute_locations(arm.random_points[position])
    for link in pos:
        draw_line(x1, y1, org_x + link[0], org_y - link[1])
        x1 = org_x + link[0]
        y1 = org_y - link[1]

    set_stroke_color(0, 0, 0)
    set_fill_color(0, 0, 0)
    for ob in arm.obstacles:
        verts = []
        for coord in ob.exterior.coords:
            verts.append((org_x + coord[0], org_y - coord[1]))
        draw_polygon(verts)


# Steps through solution path
def step():
    global position, answer
    index = answer.path.index(position)
    if index < len(answer.path) - 1:
        position = answer.path[index + 1]


if __name__ == '__main__':
    ## MAP 1:
    # p1 = Polygon([(100, 100), (100, 125), (125, 125), (125, 100)])
    # p2 = Polygon([(-100, 100), (-125, 100), (-125, 125), (-100, 125)])
    # p3 = Polygon([(-100, -100), (-100, -125), (-125, -125), (-125, -100)])
    # p4 = Polygon([(100, -100), (125, -100), (125, -125), (100, -125)])

    ## MAP 2:
    p1 = Polygon([(50, 50), (50, 125), (125, 125), (125, 50)])
    p2 = Polygon([(-50, 50), (-125, 50), (-125, 125), (-50, 125)])
    p3 = Polygon([(-50, -50), (-50, -125), (-125, -125), (-125, -50)])
    p4 = Polygon([(50, -50), (125, -50), (125, -125), (50, -125)])

    ## MAP 3:
    # p1 = Polygon([(100, 100), (100, 125), (125, 125), (125, 100)])
    # p2 = Polygon([(-25, -125), (-125, -125), (-125, 125), (-25, 125)])
    # p3 = Polygon([(-25, -100), (-25, -125), (0, -125), (0, -100)])
    # p4 = Polygon([(100, -100), (125, -100), (125, -125), (100, -125)])

    ## 2R:
    # start_angles = (math.pi / 2, 5 * math.pi / 3)
    # goal_angles = (3 * math.pi / 2, math.pi / 3)
    # arm_lengths = [50, 50]

    ## 3R:
    # start_angles = (math.pi / 2, 5 * math.pi / 3, math.pi / 2)
    # goal_angles = (3 * math.pi / 2, math.pi / 3, 3 * math.pi / 2)
    # arm_lengths = [50, 50, 50]

    ## 3R:
    start_angles = (math.pi / 2, 5 * math.pi / 3, math.pi / 2)
    goal_angles = (3 * math.pi / 2, math.pi / 3, 3 * math.pi / 2)
    arm_lengths = [65, 50, 50]

    ## 4R:
    # start_angles = (math.pi / 2, 5 * math.pi / 3, math.pi / 2, 1)
    # goal_angles = (3 * math.pi / 2, math.pi / 3, 3 * math.pi / 2, 3)
    # arm_lengths = [25, 100, 50, 50]

    arm = ArmRobot(arm_lengths, start_angles, goal_angles, [p1, p2, p3, p4], 10000, 10)
    road_map = arm.road_map
    answer = astar_search.astar_search(arm, arm.angle_distance)
    print(answer)
    # print(arm.road_map)

    # If a solution is found, draws answer, otherwise draws configuration space
    if len(answer.path) > 0:
        position = answer.path[0]
        start_graphics(draw_answer, width=width, height=height)
    else:
        start_graphics(draw_config, width=width, height=height)
