"""
Anna Matusewicz
CS 76 Prof. Balkcom
11/16/17
"""

from cs1lib import *
from planarsim import *
from shapely.geometry import Polygon, Point
from MobileRobot import MobileRobot

radius = 1


class TrajectoryView:
    def __init__(self, robot, scale):
        self.robot = robot
        self.tree = robot.tree
        self.path = robot.backchain()
        self.obstacles = robot.obstacles
        self.scale = scale
        self.ans = robot.start_state

    # Draws start and end points, RRT, path, and obstacles
    def draw(self):
        # Draws RRT
        set_stroke_color(0, 0, 0)
        set_fill_color(0, 0, 0)
        set_stroke_width(1)
        for node in self.tree:
            for neighbor in self.tree[node]:
                draw_line(node[0] * self.scale, node[1] * self.scale, neighbor[0] * self.scale, neighbor[1] * self.scale)

        # Draws Obstacles
        for ob in self.obstacles:
            verts = []
            for coord in ob.exterior.coords:
                verts.append((coord[0] * self.scale, coord[1] * self.scale))
            draw_polygon(verts)

        # If solvable, draws solution path
        if self.robot.solved:
            set_stroke_color(1, 0, 0)
            set_fill_color(1, 0, 0)
            set_stroke_width(5)
            next_node = self.path[self.path.index(self.ans) + 1]
            # print(next_node)
            draw_line(self.ans[0] * self.scale, self.ans[1] * self.scale, next_node[0] * self.scale, next_node[1] * self.scale)
            self.step()

        # Draws start and end dots
        draw_ellipse(self.robot.start_state[0] * self.scale, self.robot.start_state[1] * self.scale, 5, 5)
        draw_ellipse(self.robot.goal_state[0] * self.scale, self.robot.goal_state[1] * self.scale, 5, 5)

    # Steps through solution path
    def step(self):
        index = self.path.index(self.ans)
        if index < len(self.path) - 2:
            self.ans = self.path[index + 1]


# Method for cs1lib
def display():
    tview.draw()


if __name__ == '__main__':
    ## TEST 1:
    # p1 = Polygon([(125, 125), (150, 125), (125, 150)])
    # p2 = Polygon([(75, 125), (50, 125), (50, 150), (75, 150)])
    # p3 = Polygon([(75, 75), (50, 75), (50, 50), (75, 50)])
    # p4 = Polygon([(125, 75), (150, 75), (150, 50), (125, 50)])
    #
    # rob = MobileRobot((100, 100, 0), (150, 150), [p1, p2, p3, p4])
    # ans = rob.rrt(1000, 10)

    ## TEST 2:
    p1 = Polygon([(75, 125), (75, 75), (125, 75), (125, 125), (135, 125), (135, 65), (65, 65), (65, 125)])
    p2 = Polygon([(50, 150), (50, 100), (45, 100), (45, 155), (155, 155), (155, 100), (150, 100), (150, 150)])
    p3 = Polygon([(0, 50), (175, 50), (175, 45), (0, 45)])

    rob = MobileRobot((100, 100, 0), (100, 10), [p1, p2, p3])
    ans = rob.rrt(1500, 10)

    ## TEST 3:
    # p1 = Polygon([(0, 50), (150, 50), (150, 45), (0, 45)])
    # p2 = Polygon([(50, 100), (200, 100), (200, 95), (50, 95)])
    # p3 = Polygon([(0, 150), (150, 150), (150, 145), (0, 145)])
    # p4 = Polygon([(50, 200), (200, 200), (200, 195), (50, 195)])
    #
    # rob = MobileRobot((25, 25, 0), (25, 160), [p1, p2, p3, p4])
    # ans = rob.rrt(2000, 10)


    print(ans)
    tview = TrajectoryView(rob, 4)
    start_graphics(display, width=800, height=800)
