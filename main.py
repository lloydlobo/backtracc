import copy
import heapq
import time
from enum import Enum
from typing import List, Tuple


class TileKind(Enum):
    OPEN = 0
    OBSTACLE = 1
    VISITED = 2


class PathFinderAlgorithm(Enum):
    BACKTRACKING = 0
    A_STAR = 1
    DIJKSTRA = 2


class PathFinder:
    def __init__(self, grid: List[List[int]], start: Tuple[int, int], end: Tuple[int, int]):
        self.grid_before, self.grid_after = grid, copy.deepcopy(grid)
        self.n_rows, self.n_cols = len(grid), len(grid[0])
        self.path_list = [None] * (self.n_rows * self.n_cols)
        self.start = start
        self.end = end
        self.directions = {'N': (-1, 0), 'S': (1, 0), 'E': (0, -1), 'W': (0, 1)}
        self.__ptime_start, self.__ptime_end = None, None

    def find_path(self, algo: PathFinderAlgorithm = PathFinderAlgorithm.DIJKSTRA, verbose=False) -> bool | None:
        if verbose:
            self.debug_pretty_maze(self.grid_before)
        if verbose and isinstance(algo, PathFinderAlgorithm):
            print(f'Searching...\n\tSource: {self.start}\n\tDestination: {self.end}\n\tAlgorithm: {repr(algo.name)}')

        self.__perf_start_time()
        found: bool | None = self.__solve_maze(algo)
        self.__perf_stop_time()

        if verbose:
            path = f'{self.start} {", ".join(self.list_path_directions())} {self.end}'
            print(f'Found path:\n\t{path}') if found else print(f'Failed to find path:\n\t{path}')
        if verbose:
            print(f'Elapsed time: {repr(round(self.get_elapsed_perf_counter(), 6))}')
            print(f'Valid moves: {repr(self.count_valid_moves())}')
            self.debug_pretty_maze(self.grid_after)

        return found

    def backtrack(self, x: int, y: int, path_len: int) -> bool:
        """Back recursion"""
        if (x, y) == self.end:
            self.__mark_visited(x, y)
            self.path_list[path_len] = '_End'
            return True
        if not self.is_valid(x, y): return False
        self.__mark_visited(x, y)
        for direction, (dx, dy) in self.directions.items():
            nx, ny = x + dx, y + dy
            if self.backtrack(nx, ny, path_len + 1):
                self.path_list[path_len], found = direction, True
                return found
        self.__unmark_visited(x, y)
        return False

    def dijkstra(self):
        pq = [(0, self.start)]
        visited = set()

        while pq:
            cost, cur = heapq.heappop(pq)
            if cur == self.end:
                return True
            if cur in visited:
                continue
            visited.add(cur)
            for direction, (dx, dy) in self.directions.items():
                nxt_node = (cur[0] + dx, cur[1] + dy)
                if self.is_valid(*nxt_node):
                    heapq.heappush(pq, (cost + 1, nxt_node))
                    self.path_list.append(direction)
                    self.__mark_visited(nxt_node[0], nxt_node[1])
        return False

    def get_cost_heuristic(self, node: tuple[int, int]):
        # return 1  # it works even if we return cost as 1, but expensive as it is not precise.
        return abs(node[0] - self.__end_totuple()[1]) + abs(node[1] - self.__end_totuple()[1])

    def a_star(self):
        pq = [(0, self.start)]
        visited = set()
        cost_map = {}
        cost_lst = []
        counter = -1

        while pq:
            counter += 1
            _, cur = heapq.heappop(pq)
            if cur == self.end:
                return True
            if cur in visited:
                continue
            visited.add(cur)
            for direction, (dx, dy) in self.directions.items():
                nxt_node = (cur[0] + dx, cur[1] + dy)
                if self.is_valid(*nxt_node):
                    cost = self.get_cost_heuristic(nxt_node)
                    t = dict(id_vs_i=(len(cost_lst), counter), cur_nxt=(cur, nxt_node), cost=cost,
                             n_seen=len(visited))
                    cost_map[counter] = t
                    cost_lst.append(t)
                    heapq.heappush(pq, (cost, nxt_node))
                    self.__mark_visited(nxt_node[0], nxt_node[1])
                    self.path_list.append(direction)
        return False

    def is_valid(self, x, y) -> bool:
        return 0 <= x < self.n_rows and 0 <= y < self.n_cols and self.grid_after[x][y] == TileKind.OPEN.value

    def list_path_directions(self):
        return [p for p in self.path_list if p and p in self.directions]

    def count_valid_moves(self):
        return sum(1 for p in self.path_list if p and p in self.directions)

    def get_elapsed_perf_counter(self) -> float:
        return self.__ptime_end - self.__ptime_start

    def __solve_maze(self, algo):
        found: bool | None = None
        path_len = 0
        self.path_list[path_len] = '_Start'
        if algo == PathFinderAlgorithm.BACKTRACKING:
            found = self.backtrack(x=self.start[0], y=self.start[1], path_len=(path_len + 1))
        elif algo == PathFinderAlgorithm.DIJKSTRA:
            found = self.dijkstra()
        elif algo == PathFinderAlgorithm.A_STAR:
            found = self.a_star()
        else:
            assert False, f'unimplemented {repr(algo)}'
        return found

    def debug_pretty_maze(self, grid) -> None:
        legend = {'OPEN': '_', 'OBSTACLE': '.', 'VISITED': '*'}
        (xa, ya), (xb, yb) = self.start, self.end
        maze = [[None] * self.n_cols for _ in range(self.n_rows)]
        for i, r in enumerate(grid):
            for j, c in enumerate(r):
                maze[i][j] = legend.get(TileKind(c).name, None)
        maze[xa][ya], maze[xb][yb] = 'S', 'E'
        for r in maze:
            print('\t'.join(map(str, r)))

    def __start_totuple(self) -> Tuple[int, int]:
        return self.start

    def __end_totuple(self) -> Tuple[int, int]:
        return self.end

    def __mark_visited(self, x, y):
        self.grid_after[x][y] = TileKind.VISITED.value

    def __unmark_visited(self, x, y):
        self.grid_after[x][y] = TileKind.OPEN.value

    def __perf_start_time(self):
        self.__ptime_start = time.perf_counter()

    def __perf_stop_time(self):
        self.__ptime_end = time.perf_counter()


maze = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 1, 1, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 1, 1, 1, 0],
]

maze_start, maze_end = (0, 0), ((len(maze) - 1), (len(maze[0]) - 1))

path_finder = PathFinder(maze, maze_start, maze_end)
_found = path_finder.find_path(algo=PathFinderAlgorithm.BACKTRACKING, verbose=True)
# _found = path_finder.find_path(algo=PathFinderAlgorithm.DIJKSTRA, verbose=True)
# _found = path_finder.find_path(algo=PathFinderAlgorithm.A_STAR, verbose=True)
