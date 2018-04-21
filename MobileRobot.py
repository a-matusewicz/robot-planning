"""
Anna Matusewicz
CS 76 Prof. Balkcom
11/16/17
"""

from random import random, seed
from shapely.geometry import Polygon, Point, LineString
from planarsim import *
from cs1lib import *
# from display_planar import TrajectoryView, display


class MobileRobot:
    def __init__(self, start_state, goal_state, obstacles):
        seed(1)
        self.start_state = start_state
        self.goal_state = goal_state  # Just (x,y)
        self.obstacles = obstacles
        self.tree = {self.start_state: []}
        self.bp = {self.start_state: None}
        self.solved = False

    def __str__(self):
        return "MobileRobot"
    
    # inputs number of random points allowed and duration for branching
    def rrt(self, k, duration):
        md = duration
        
        # For the number of random point allowed
        for i in range(k):
            x = random() * 200
            y = random() * 200
            rand_point = Point(x, y)

            dist = 100000
            exp_point = self.start_state

            # Find the closest point to the random point that doesn't cross an obstacle
            for point in self.tree:
                p = Point(point[0], point[1])
                new_dist = p.distance(rand_point)
                if new_dist < dist and not self.collides(point, (x, y)):
                    dist = new_dist
                    exp_point = point

            # Check if expansion point is nearby goal
            euc = self.get_euclidean(exp_point)
            if euc < md:
                # If so, shrink duration
                print("Near at {}".format(exp_point))
                md = int(euc)

            # For each control
            for j in range(len(controls_rs[0:])):
                # Get correct transformation
                t = transform_from_config(exp_point)

                # Preform a single action for that control
                act = single_action(t, controls_rs[j], md)

                # Get the new point to add to the tree
                new_point = config_from_transform(act)

                # If not already expanded and valid
                if new_point not in self.tree and not self.collides(exp_point, new_point):
                    self.tree[exp_point].append(new_point)

                    # Add Backpointer
                    self.bp[new_point] = exp_point

                    # If robot is close enough to the goal
                    if self.get_euclidean(new_point) < 2:
                        print("Found at {}".format(new_point))
                        self.tree[new_point] = [self.goal_state]
                        self.bp[self.goal_state] = new_point
                        self.solved = True
                        return new_point
                    else:
                        self.tree[new_point] = []

            md = duration

        return "Not Found"

    # Detects collision with boundry or obstacle
    def collides(self, point1, point2):
        if not(0 <= point1[0] < 200 and 0 <= point1[1] < 200):
            return True

        # Tests path between two points
        l = LineString([(point1[0], point1[1]), (point2[0], point2[1])])
        for obs in self.obstacles:
            if l.intersects(obs):
                return True

        return False

    # Gets the euclidean distance between point and the goal state
    def get_euclidean(self, point):
        p = Point(point[0], point[1])
        g = Point(self.goal_state[0], self.goal_state[1])
        return g.distance(p)

    # Backchains to find solution path
    def backchain(self):
        if self.solved:
            result = []
            current = self.goal_state
            while current:
                result.append(current)
                current = self.bp[current]

            result.reverse()
            return result
        else:
            return None
