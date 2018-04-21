"""
Anna Matusewicz
CS 76 Prof. Balkcom
9/27/17
"""

from SearchSolution import SearchSolution
from heapq import heappush, heappop


class AstarNode:

    # each search node except the root has a parent node
    # and all search nodes wrap a state object
    def __init__(self, state, heuristic, parent=None, transition_cost=0):
        # you write this part
        self.state = state
        self.heuristic = heuristic
        self.parent = parent
        self.transition_cost = transition_cost

    def priority(self):
        # How many moves to that state + heuristic results
        return self.transition_cost + self.heuristic

    # comparison operator,
    # needed for heappush and heappop to work with AstarNodes:
    def __lt__(self, other):
        return self.priority() < other.priority()


# take the current node, and follow its parents back
#  as far as possible. Grab the states from the nodes,
#  and reverse the resulting list of states.
def backchain(node):
    result = []
    current = node
    while current:
        result.append(current.state)
        current = current.parent

    result.reverse()
    return result


def astar_search(search_problem, heuristic_fn):
    # print("Start Astar")
    # I'll get you started:
    start_node = AstarNode(search_problem.start_state, heuristic_fn(search_problem.start_state))
    pqueue = []
    heappush(pqueue, start_node)
    nodes_visited = 0

    solution = SearchSolution(search_problem, "Astar with heuristic " + heuristic_fn.__name__)

    visited_cost = {start_node.state: 0}

    # you write the rest:
    while len(pqueue) > 0:
        nodes_visited += 1
        node = heappop(pqueue)

        # If the node is the solution state
        if search_problem.same_test(node.state):
            solution.nodes_visited = nodes_visited
            solution.path = backchain(node)
            solution.cost = node.transition_cost
            return solution

        children = search_problem.get_successors(node.state)
        for child in children:
            new_node = AstarNode(child, heuristic_fn(child), node, node.transition_cost)

            # If new_node is not the same as it's parent node
            if not search_problem.same_test(node.state, new_node.state):
                new_node.transition_cost += 1

            # If new_node has not been added to pqueue yet
            if new_node.state not in visited_cost:
                heappush(pqueue, new_node)
                visited_cost[new_node.state] = new_node.transition_cost
            # If new_node was added to pqueue previously with a higher transition_cost
            elif new_node.transition_cost < visited_cost[new_node.state]:
                heappush(pqueue, new_node)
                visited_cost[new_node.state] = new_node.transition_cost

    # If the search fails
    solution.nodes_visited = nodes_visited
    return solution
