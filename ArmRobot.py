"""
Anna Matusewicz
CS 76 Prof. Balkcom
11/16/17
"""

import math
from random import random, seed
from shapely.geometry import Polygon, LineString
from annoy import AnnoyIndex


class ArmRobot:
    def __init__(self, arm_lengths, joint_angles, goal_state, obstacles, max_rand=10000, num_neighbors=10):
        seed(1)
        self.arm_lengths = arm_lengths
        self.start_state = 0
        self.goal_state = 1
        self.obstacles = obstacles
        self.random_points = [joint_angles, goal_state]
        self.collision = False
        self.road_map = self.build_roadmap(max_rand, num_neighbors)

    def __str__(self):
        return "ArmRobot"

    # Computes joint locations from a tuple of angles
    def compute_locations(self, angles):
        locations = []
        x1 = 0
        y1 = 0
        t1 = 0
        for i in range(len(self.arm_lengths)):
            x2 = x1 + self.arm_lengths[i] * math.cos(t1 + angles[i])
            y2 = y1 + self.arm_lengths[i] * math.sin(t1 + angles[i])
            locations.append((x2, y2))
            x1 = x2
            y1 = y2
            t1 += angles[i] % (2 * math.pi)

        return locations

    # Tests if placement or movement causes a collision
    def collides(self, angles, new_angles=None):
        locs = self.compute_locations(angles)
        # Movement
        if new_angles:
            new_locs = self.compute_locations(new_angles)
            for i in range(len(new_locs)):
                arm = LineString([locs[i], new_locs[i]])

                for obs in self.obstacles:
                    if arm.intersects(obs):
                        return True

        # Placement
        else:
            start = (0, 0)
            for l in locs:

                arm = LineString([start, l])
                start = l

                for obs in self.obstacles:

                    if arm.intersects(obs):
                        self.collision = True
                        return True

        self.collision = False

        return False

    # Inputs include max random positions and max number of neighbors
    def build_roadmap(self, maxi, neighbors):
        self.random_points += self.get_vertices(maxi)

        # Uses Annoy to build index using euclidean distance
        road_map = {}
        t = AnnoyIndex(len(self.random_points[0]), "euclidean")
        for i in range(len(self.random_points)):
            v = self.random_points[i]
            t.add_item(i, v)

        t.build(10)  # 10 trees

        # Finds the nearest neighbors for each random point
        for i in range(len(self.random_points)):
            neigh = t.get_nns_by_item(i, neighbors)

            if i not in road_map:
                road_map[i] = []

            for n in neigh:
                # If a different point and does not collide
                if not n == i and not self.collides(self.random_points[i], self.random_points[n]):
                    # Creates an undirected edge
                    if n not in road_map[i]:
                        road_map[i].append(n)
                    if n not in road_map:
                        road_map[n] = []
                    if i not in road_map[n]:
                        road_map[n].append(i)

        return road_map

    # Gets a set amount of random robot arm placements
    def get_vertices(self, maxi):
        rand = []
        for i in range(maxi):
            angle = []
            for j in range(len(self.random_points[0])):
                angle.append(2 * math.pi * random())
            if not self.collides(angle):
                rand.append(tuple(angle))

        return rand

    # Gets neighbors for Astar
    def get_successors(self, angle):
        return self.road_map[angle]

    # Tests if angle is the same as another or the solution
    def same_test(self, angle1, angle2=None):
        if angle2:
            return angle1 == angle2
        else:
            return angle1 == 1

    # Tests the distance between two angles between 0 and 2pi
    def angle_distance(self, angle):
        dist = 0
        for i in range(len(self.random_points[angle])):
            dist += abs(self.random_points[angle][i] - self.random_points[1][i]) % (2 * math.pi)

        return dist
