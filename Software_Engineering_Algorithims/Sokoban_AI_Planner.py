"""
This project implements intelligent path planning strategies to solve Sokoban puzzles using the A* family of algorithms. 
It features custom heuristics that account for deadlocks (like corner traps and wall-blocks) and assigns storage targets to boxes dynamically to improve path efficiency. The planner supports:

Weighted A* (for fast, suboptimal solutions)

Iterative A* (gradually improving quality)

Greedy Best-First Search

Custom corner- and wall-aware heuristics

Classic Manhattan Distance heuristic as a baseline

The project was developed using Python and demonstrates advanced search techniques introduced in CSC384: Introduction to Artificial Intelligence.

"""

import os
import math
from search import *
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS

def heur_alternate(state):
    """
    Improved heuristic that checks for:
    - Boxes stuck in corners or against walls with no storage
    - Trapped configurations (no way to push the box)
    - Assigns each box to its closest available storage to avoid overlaps
    """
    total_distance = 0
    available_storage = set(state.storage)
    box_set = set(state.boxes)
    obstacle_set = set(state.obstacles)
    corners = [
        (0, state.height - 1), (state.width - 1, 0),
        (state.width - 1, state.height - 1), (0, 0)
    ]

    for box in state.boxes:
        # Hard deadlock: Box in a corner with no storage
        if box in corners and box not in available_storage:
            return float('inf')

        # Wall traps with no aligned storage
        if (box[0] in [0, state.width - 1] and not any(s[0] == box[0] for s in state.storage)) or \
           (box[1] in [0, state.height - 1] and not any(s[1] == box[1] for s in state.storage)):
            return float('inf')

        # Check if box is stuck (both horizontal and vertical directions blocked)
        horiz_blocked = (
            box[0] in [0, state.width - 1] or
            (box[0] - 1, box[1]) in box_set or (box[0] - 1, box[1]) in obstacle_set or
            (box[0] + 1, box[1]) in box_set or (box[0] + 1, box[1]) in obstacle_set
        )
        vert_blocked = (
            box[1] in [0, state.height - 1] or
            (box[0], box[1] - 1) in box_set or (box[0], box[1] - 1) in obstacle_set or
            (box[0], box[1] + 1) in box_set or (box[0], box[1] + 1) in obstacle_set
        )
        if box not in available_storage and horiz_blocked and vert_blocked:
            return float('inf')

        # Assign closest unclaimed storage
        min_distance = float('inf')
        closest_storage = None
        for storage in available_storage:
            distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if distance < min_distance:
                min_distance = distance
                closest_storage = storage

        if closest_storage is None:
            return float('inf')
        available_storage.remove(closest_storage)
        total_distance += min_distance

    return total_distance

def heur_zero(state):
    """Trivial heuristic for uniform-cost search (baseline)."""
    return 0

def heur_manhattan_distance(state):
    """Sum of Manhattan distances from boxes to the closest storage."""
    total_distance = 0
    for box in state.boxes:
        min_distance = min(abs(box[0] - s[0]) + abs(box[1] - s[1]) for s in state.storage)
        total_distance += min_distance
    return total_distance

def fval_function(sN, weight):
    """Custom f-value: f(n) = g(n) + w * h(n)"""
    return sN.gval + weight * sN.hval

def weighted_astar(initial_state, heur_fn, weight, timebound):
    """Performs Weighted A* Search with a custom heuristic."""
    search = SearchEngine('custom', 'default')
    search.init_search(initial_state, goal_fn=sokoban_goal_state,
                       heur_fn=heur_fn,
                       fval_function=lambda sN: fval_function(sN, weight))
    return search.search(timebound)

def iterative_astar(initial_state, heur_fn, weight=5, timebound=5):
    """
    Runs iterative A* with decreasing weights for improving solution quality.
    """
    best_solution = None
    best_cost = float('inf')
    remaining_time = timebound

    while remaining_time > 0:
        start = os.times()[0]
        result, stats = weighted_astar(best_solution or initial_state, heur_fn, weight, remaining_time)
        elapsed = os.times()[0] - start
        remaining_time -= elapsed

        if result:
            cost = result.gval + weight * heur_fn(result)
            if cost < best_cost:
                best_solution = result
                best_cost = cost
        else:
            break
        weight = max(2, weight * 0.5)

    return best_solution, stats

def iterative_gbfs(initial_state, heur_fn, timebound=5):
    """
    Iterative Greedy Best-First Search. Improves the best g(n) found within time limit.
    """
    search = SearchEngine('best_first', 'default')
    search.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)

    start = os.times()[0]
    best_solution, stats = search.search(timebound)
    best_gval = best_solution.gval if best_solution else float('inf')
    remaining = timebound - (os.times()[0] - start)

    while remaining > 0:
        candidate, stats = search.search(remaining, [best_gval, float("inf"), float("inf")])
        if candidate and candidate.gval < best_gval:
            best_solution = candidate
            best_gval = candidate.gval
        remaining = timebound - (os.times()[0] - start)

    return best_solution, stats
