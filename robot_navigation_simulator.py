"""
╔══════════════════════════════════════════════════════════════════╗
║       ROBOT NAVIGATION SIMULATOR - AI Search Algorithms         ║
║  Algorithms: BFS | DFS | A* (Dijkstra variant)                  ║
║  Tech Stack: Python + Pygame + NumPy                            ║
╚══════════════════════════════════════════════════════════════════╝

CONTROLS:
  SPACE       - Start / Pause simulation
  R           - Reset grid (keep obstacles)
  C           - Clear ALL obstacles
  1           - Switch to BFS
  2           - Switch to DFS
  3           - Switch to A* (heuristic search)
  + / =       - Increase simulation speed
  - / _       - Decrease simulation speed
  LEFT CLICK  - Place / Remove obstacle
  S           - Place Start (then click)
  G           - Place Goal (then click)

INSTALL:
  pip install pygame numpy

RUN:
  python robot_navigation_simulator.py
"""

import pygame
import numpy as np
import sys
import math
import time
from collections import deque
import heapq

# ─────────────────────────────────────────────
#  CONSTANTS & CONFIGURATION
# ─────────────────────────────────────────────

GRID_COLS = 20
GRID_ROWS = 20
CELL_SIZE  = 36          # pixels per cell
PANEL_W    = 260         # right-side info panel width

WIN_W = GRID_COLS * CELL_SIZE + PANEL_W
WIN_H = GRID_ROWS * CELL_SIZE

FPS = 60

# Speed settings: steps per second
SPEEDS = [1, 2, 4, 8, 16, 32, 64]
DEFAULT_SPEED_IDX = 3

# ── Colour Palette ──────────────────────────
C_BG         = (15,  20,  35)
C_GRID       = (30,  40,  65)
C_EMPTY      = (22,  30,  52)
C_OBSTACLE   = (20,  20,  28)
C_OBSTACLE_B = (50,  50,  80)   # obstacle border accent
C_START      = (0,  200, 120)
C_GOAL       = (255, 200,  30)
C_EXPLORED   = (30,  80, 160)
C_EXPLORED2  = (20,  55, 110)
C_PATH       = (0,  220, 255)
C_ROBOT      = (60,  180, 255)
C_ROBOT_GLOW = (20,  80, 140)
C_PANEL_BG   = (10,  14,  25)
C_PANEL_LINE = (30,  45,  80)
C_TEXT_H     = (200, 220, 255)
C_TEXT       = (130, 160, 210)
C_TEXT_DIM   = (60,  80, 120)
C_ACCENT_BFS = (0,  180, 255)
C_ACCENT_DFS = (200, 80, 255)
C_ACCENT_AST = (255, 160,  30)
C_SUCCESS    = (0,  230, 120)
C_FAIL       = (255,  60,  60)

# ── Cell States ─────────────────────────────
EMPTY    = 0
OBSTACLE = 1
START    = 2
GOAL     = 3

# ── Simulation States ────────────────────────
SIM_IDLE     = "IDLE"
SIM_RUNNING  = "RUNNING"
SIM_DONE     = "DONE"
SIM_NO_PATH  = "NO PATH"
SIM_PAUSED   = "PAUSED"


# ─────────────────────────────────────────────
#  GRID MANAGEMENT
# ─────────────────────────────────────────────

def create_grid(rows, cols):
    """Create a blank 2D NumPy grid. All cells start as EMPTY."""
    return np.zeros((rows, cols), dtype=int)


def set_default_map(grid, rows, cols):
    """
    Place a default start, goal, and some walls so the
    simulator is immediately interesting on launch.
    Returns (start_pos, goal_pos).
    """
    grid[:] = EMPTY
    start = (rows // 2, 1)
    goal  = (rows // 2, cols - 2)

    # Vertical wall in the middle with a gap
    mid = cols // 2
    for r in range(rows):
        if r != rows // 2 - 1 and r != rows // 2 and r != rows // 2 + 1:
            grid[r][mid] = OBSTACLE

    # A second partial wall
    mid2 = cols * 3 // 4
    for r in range(2, rows - 2):
        if r < rows // 2 - 2 or r > rows // 2 + 2:
            grid[r][mid2] = OBSTACLE

    grid[start[0]][start[1]] = START
    grid[goal[0]][goal[1]]   = GOAL
    return start, goal


def neighbors(pos, rows, cols, grid):
    """Return valid 4-directional neighbours (no diagonals)."""
    r, c = pos
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    result = []
    for dr, dc in dirs:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            if grid[nr][nc] != OBSTACLE:
                result.append((nr, nc))
    return result


# ─────────────────────────────────────────────
#  SEARCH ALGORITHMS
# ─────────────────────────────────────────────

def bfs(grid, start, goal, rows, cols):
    """
    Breadth-First Search.
    Guarantees shortest path in an unweighted graph.
    Returns (visited_order, parent_map)
    """
    queue   = deque([start])
    visited = {start: None}   # maps node → parent
    order   = []              # exploration order

    while queue:
        node = queue.popleft()
        order.append(node)
        if node == goal:
            break
        for nb in neighbors(node, rows, cols, grid):
            if nb not in visited:
                visited[nb] = node
                queue.append(nb)

    return order, visited


def dfs(grid, start, goal, rows, cols):
    """
    Depth-First Search (iterative with explicit stack).
    Does NOT guarantee shortest path.
    Returns (visited_order, parent_map)
    """
    stack   = [start]
    visited = {start: None}
    order   = []

    while stack:
        node = stack.pop()
        order.append(node)
        if node == goal:
            break
        for nb in neighbors(node, rows, cols, grid):
            if nb not in visited:
                visited[nb] = node
                stack.append(nb)

    return order, visited


def heuristic(a, b):
    """Manhattan distance heuristic for A*."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal, rows, cols):
    """
    A* Search using Manhattan distance heuristic.
    Guarantees shortest path; more efficient than BFS via heuristic.
    Returns (visited_order, parent_map)
    """
    # Priority queue: (f_score, node)
    open_heap = []
    heapq.heappush(open_heap, (0, start))

    g_score = {start: 0}
    visited = {start: None}
    order   = []
    in_open = {start}

    while open_heap:
        _, node = heapq.heappop(open_heap)
        in_open.discard(node)
        order.append(node)

        if node == goal:
            break

        for nb in neighbors(node, rows, cols, grid):
            tentative_g = g_score[node] + 1
            if nb not in g_score or tentative_g < g_score[nb]:
                g_score[nb] = tentative_g
                f = tentative_g + heuristic(nb, goal)
                visited[nb] = node
                if nb not in in_open:
                    heapq.heappush(open_heap, (f, nb))
                    in_open.add(nb)

    return order, visited


def reconstruct_path(visited, start, goal):
    """
    Walk the parent map backwards from goal → start.
    Returns the path as a list [start, ..., goal].
    Returns empty list if goal was never reached.
    """
    if goal not in visited:
        return []
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = visited[node]
    path.reverse()
    if path[0] != start:
        return []
    return path


ALGORITHMS = {
    "BFS": bfs,
    "DFS": dfs,
    "A*" : astar,
}

ALGO_COLORS = {
    "BFS": C_ACCENT_BFS,
    "DFS": C_ACCENT_DFS,
    "A*" : C_ACCENT_AST,
}

ALGO_NAMES = ["BFS", "DFS", "A*"]


# ─────────────────────────────────────────────
#  DRAWING HELPERS
# ─────────────────────────────────────────────

def cell_rect(r, c):
    """Return pygame.Rect for a grid cell."""
    return pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)


def draw_rounded_rect(surf, color, rect, radius=4, border=0, border_color=None):
    pygame.draw.rect(surf, color, rect, border_radius=radius)
    if border and border_color:
        pygame.draw.rect(surf, border_color, rect, border, border_radius=radius)


def lerp_color(c1, c2, t):
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


def draw_grid(surf, grid, rows, cols, explored_set, path_set,
              robot_pos, goal_pos, start_pos, tick):
    """Render the full grid each frame."""

    # Background
    surf.fill(C_BG)

    for r in range(rows):
        for c in range(cols):
            rect  = cell_rect(r, c)
            inner = rect.inflate(-2, -2)
            cell  = grid[r][c]
            pos   = (r, c)

            if cell == OBSTACLE:
                draw_rounded_rect(surf, C_OBSTACLE, inner, 3)
                # subtle accent on border
                pygame.draw.rect(surf, C_OBSTACLE_B, inner, 1, border_radius=3)

            elif pos in path_set and pos != start_pos and pos != goal_pos:
                # Animated path pulse
                pulse = 0.7 + 0.3 * math.sin(tick * 0.12 + c * 0.3)
                color = lerp_color(C_PATH, (180, 255, 255), pulse * 0.4)
                draw_rounded_rect(surf, color, inner, 4)
                # Bright dot centre
                cx, cy = inner.centerx, inner.centery
                pygame.draw.circle(surf, (220, 255, 255), (cx, cy), 3)

            elif pos in explored_set and pos != start_pos and pos != goal_pos:
                depth_shade = min(1.0, len(explored_set) / (rows * cols))
                color = lerp_color(C_EXPLORED2, C_EXPLORED, depth_shade)
                draw_rounded_rect(surf, color, inner, 3)

            else:
                draw_rounded_rect(surf, C_EMPTY, inner, 3)

            # Grid lines
            pygame.draw.rect(surf, C_GRID, rect, 1)

    # ── Start cell ──
    if start_pos:
        r, c = start_pos
        inner = cell_rect(r, c).inflate(-4, -4)
        draw_rounded_rect(surf, C_START, inner, 5)
        # "S" label
        _draw_cell_label(surf, inner, "S", (10, 30, 20))

    # ── Goal cell ──
    if goal_pos:
        r, c = goal_pos
        inner = cell_rect(r, c).inflate(-4, -4)
        pulse = 0.7 + 0.3 * math.sin(tick * 0.1)
        color = lerp_color(C_GOAL, (255, 230, 100), pulse)
        draw_rounded_rect(surf, color, inner, 5)
        _draw_cell_label(surf, inner, "G", (40, 30, 0))

    # ── Robot ──
    if robot_pos:
        r, c = robot_pos
        rect = cell_rect(r, c)
        # Glow ring
        glow = rect.inflate(6, 6)
        pygame.draw.rect(surf, C_ROBOT_GLOW, glow, border_radius=8)
        inner = rect.inflate(-4, -4)
        draw_rounded_rect(surf, C_ROBOT, inner, 6)
        # Eye dots
        ex = inner.centerx - 4
        ey = inner.centery - 2
        pygame.draw.circle(surf, (10, 20, 40), (ex, ey), 3)
        pygame.draw.circle(surf, (10, 20, 40), (ex + 8, ey), 3)
        # Mouth
        pygame.draw.line(surf, (10, 20, 40),
                         (inner.centerx - 4, inner.centery + 4),
                         (inner.centerx + 4, inner.centery + 4), 2)


def _draw_cell_label(surf, rect, text, color):
    """Draw a small bold letter centred in a cell."""
    font = pygame.font.SysFont("consolas", 13, bold=True)
    img  = font.render(text, True, color)
    surf.blit(img, img.get_rect(center=rect.center))


# ─────────────────────────────────────────────
#  PANEL DRAWING
# ─────────────────────────────────────────────

def draw_panel(surf, state, algo_name, steps, path_len,
               explored_count, speed_idx, place_mode, fonts):
    """Draw the right-side information panel."""
    panel_x = GRID_COLS * CELL_SIZE
    panel_rect = pygame.Rect(panel_x, 0, PANEL_W, WIN_H)
    surf.fill(C_PANEL_BG, panel_rect)
    pygame.draw.line(surf, C_PANEL_LINE, (panel_x, 0), (panel_x, WIN_H), 2)

    f_title = fonts["title"]
    f_head  = fonts["head"]
    f_body  = fonts["body"]
    f_small = fonts["small"]
    f_mono  = fonts["mono"]

    x = panel_x + 16
    y = 16

    # ── Title ──
    title = f_title.render("ROBOT NAV SIM", True, C_TEXT_H)
    surf.blit(title, (x, y))
    y += 32

    subtitle = f_small.render("AI Search Algorithm Visualizer", True, C_TEXT_DIM)
    surf.blit(subtitle, (x, y))
    y += 22

    _hline(surf, panel_x, y, PANEL_W, C_PANEL_LINE)
    y += 12

    # ── Algorithm selector ──
    lbl = f_small.render("ALGORITHM", True, C_TEXT_DIM)
    surf.blit(lbl, (x, y)); y += 18

    for i, name in enumerate(ALGO_NAMES):
        active = (name == algo_name)
        key_map = {"BFS": "1", "DFS": "2", "A*": "3"}
        color = ALGO_COLORS[name] if active else C_TEXT_DIM
        bg    = lerp_color(C_PANEL_BG, ALGO_COLORS[name], 0.12) if active else C_PANEL_BG
        bar   = pygame.Rect(x, y, PANEL_W - 32, 26)
        draw_rounded_rect(surf, bg, bar, 4,
                          border=1 if active else 0,
                          border_color=color)
        txt = f_body.render(f"[{key_map[name]}]  {name}", True, color)
        surf.blit(txt, (x + 8, y + 5))
        if active:
            pygame.draw.rect(surf, color,
                             pygame.Rect(x, y + 4, 3, 18), border_radius=2)
        y += 30

    y += 4
    _hline(surf, panel_x, y, PANEL_W, C_PANEL_LINE)
    y += 12

    # ── Status ──
    lbl = f_small.render("STATUS", True, C_TEXT_DIM)
    surf.blit(lbl, (x, y)); y += 18

    status_color = {
        SIM_IDLE:    C_TEXT_DIM,
        SIM_RUNNING: C_ACCENT_BFS,
        SIM_PAUSED:  C_ACCENT_AST,
        SIM_DONE:    C_SUCCESS,
        SIM_NO_PATH: C_FAIL,
    }.get(state, C_TEXT)

    status_icon = {
        SIM_IDLE:    "●  IDLE",
        SIM_RUNNING: "▶  RUNNING",
        SIM_PAUSED:  "⏸  PAUSED",
        SIM_DONE:    "✔  PATH FOUND",
        SIM_NO_PATH: "✖  NO PATH FOUND",
    }.get(state, state)

    if state in (SIM_RUNNING, SIM_PAUSED):
        pulse_alpha = int(128 + 127 * math.sin(pygame.time.get_ticks() * 0.005))
        status_color = lerp_color(C_TEXT_DIM, status_color, pulse_alpha / 255)

    st_img = f_head.render(status_icon, True, status_color)
    surf.blit(st_img, (x, y)); y += 28

    _hline(surf, panel_x, y, PANEL_W, C_PANEL_LINE)
    y += 12

    # ── Stats ──
    lbl = f_small.render("STATISTICS", True, C_TEXT_DIM)
    surf.blit(lbl, (x, y)); y += 18

    stats = [
        ("Steps Taken",    str(steps)),
        ("Path Length",    str(path_len) if path_len else "—"),
        ("Nodes Explored", str(explored_count)),
        ("Speed",          f"{SPEEDS[speed_idx]} step/s"),
    ]
    for label, val in stats:
        l_img = f_small.render(label, True, C_TEXT_DIM)
        v_img = f_mono.render(val, True, C_TEXT_H)
        surf.blit(l_img, (x, y))
        surf.blit(v_img, (panel_x + PANEL_W - 16 - v_img.get_width(), y))
        y += 20

    y += 4
    _hline(surf, panel_x, y, PANEL_W, C_PANEL_LINE)
    y += 12

    # ── Algorithm info box ──
    lbl = f_small.render("ABOUT THIS ALGORITHM", True, C_TEXT_DIM)
    surf.blit(lbl, (x, y)); y += 18

    algo_info = {
        "BFS": [
            "Breadth-First Search",
            "Explores layer by layer.",
            "Guarantees shortest path",
            "in unweighted graphs.",
            "Time:  O(V + E)",
            "Space: O(V)",
        ],
        "DFS": [
            "Depth-First Search",
            "Explores deep first.",
            "Does NOT guarantee",
            "shortest path.",
            "Time:  O(V + E)",
            "Space: O(V)",
        ],
        "A*": [
            "A* Search",
            "Uses heuristic (Manhattan)",
            "to guide exploration.",
            "Optimal & efficient.",
            "Time:  O(E log V)",
            "Space: O(V)",
        ],
    }
    acc_color = ALGO_COLORS[algo_name]
    for i, line in enumerate(algo_info[algo_name]):
        color = acc_color if i == 0 else (C_TEXT if i < 4 else C_TEXT_DIM)
        bold  = i == 0
        f = f_body if bold else f_small
        img = f.render(line, True, color)
        surf.blit(img, (x, y)); y += 17

    y += 4
    _hline(surf, panel_x, y, PANEL_W, C_PANEL_LINE)
    y += 12

    # ── Controls ──
    lbl = f_small.render("CONTROLS", True, C_TEXT_DIM)
    surf.blit(lbl, (x, y)); y += 18

    controls = [
        ("SPACE",  "Start / Pause"),
        ("R",      "Reset simulation"),
        ("C",      "Clear obstacles"),
        ("1/2/3",  "Switch algorithm"),
        ("+/-",    "Change speed"),
        ("Click",  "Place obstacle"),
        ("S+Click","Set start"),
        ("G+Click","Set goal"),
    ]
    for key, desc in controls:
        k_img = f_small.render(f"[{key}]", True, ALGO_COLORS[algo_name])
        d_img = f_small.render(desc, True, C_TEXT_DIM)
        surf.blit(k_img, (x, y))
        surf.blit(d_img, (x + 68, y))
        y += 17

    # ── Place mode indicator ──
    if place_mode:
        y += 6
        pm_txt = f"MODE: SET {'START' if place_mode == 'S' else 'GOAL'}"
        pm_img = f_body.render(pm_txt, True, C_SUCCESS)
        surf.blit(pm_img, (x, y))

    # ── Legend ──
    _draw_legend(surf, panel_x, WIN_H - 90, PANEL_W, fonts)


def _draw_legend(surf, panel_x, y, panel_w, fonts):
    f = fonts["small"]
    _hline(surf, panel_x, y, panel_w, C_PANEL_LINE)
    y += 8
    x = panel_x + 16
    items = [
        (C_ROBOT,    "Robot"),
        (C_START,    "Start"),
        (C_GOAL,     "Goal"),
        (C_OBSTACLE, "Wall"),
        (C_EXPLORED, "Explored"),
        (C_PATH,     "Path"),
    ]
    for i, (color, label) in enumerate(items):
        col_x = x + (i % 3) * 80
        row_y = y + (i // 3) * 20
        pygame.draw.rect(surf, color,
                         pygame.Rect(col_x, row_y + 3, 12, 12), border_radius=2)
        img = f.render(label, True, C_TEXT_DIM)
        surf.blit(img, (col_x + 16, row_y + 2))


def _hline(surf, x, y, w, color):
    pygame.draw.line(surf, color, (x + 8, y), (x + w - 8, y))


# ─────────────────────────────────────────────
#  FONT LOADER
# ─────────────────────────────────────────────

def load_fonts():
    pygame.font.init()
    try:
        return {
            "title": pygame.font.SysFont("consolas", 14, bold=True),
            "head":  pygame.font.SysFont("consolas", 13, bold=True),
            "body":  pygame.font.SysFont("consolas", 12, bold=False),
            "small": pygame.font.SysFont("consolas", 11, bold=False),
            "mono":  pygame.font.SysFont("consolas", 12, bold=True),
        }
    except Exception:
        f = pygame.font.SysFont(None, 14)
        return {k: f for k in ["title","head","body","small","mono"]}


# ─────────────────────────────────────────────
#  MAIN SIMULATION CLASS
# ─────────────────────────────────────────────

class RobotSimulator:
    """Encapsulates all simulation state and logic."""

    def __init__(self):
        self.grid       = create_grid(GRID_ROWS, GRID_COLS)
        self.start_pos, self.goal_pos = set_default_map(
            self.grid, GRID_ROWS, GRID_COLS)

        self.algo_idx   = 0
        self.algo_name  = ALGO_NAMES[self.algo_idx]

        self.state      = SIM_IDLE
        self.speed_idx  = DEFAULT_SPEED_IDX

        # Search results
        self.explore_order  = []   # list of cells in exploration order
        self.parent_map     = {}
        self.final_path     = []

        # Animation state
        self.explore_idx    = 0    # how many explored cells to draw
        self.path_idx       = 0    # how many path cells to draw
        self.robot_pos      = self.start_pos
        self.phase          = "explore"  # "explore" | "path" | "done"

        # Timing
        self.step_timer     = 0.0
        self.tick           = 0

        # Interaction
        self.place_mode     = None  # None | "S" | "G"
        self.mouse_held     = False

    # ── Properties for panel ──────────────────
    @property
    def steps(self):
        if self.phase == "explore":
            return self.explore_idx
        return self.explore_idx + self.path_idx

    @property
    def path_len(self):
        return len(self.final_path) - 1 if len(self.final_path) > 1 else 0

    @property
    def explored_count(self):
        return self.explore_idx

    # ── Simulation control ────────────────────

    def run_algorithm(self):
        """Run selected search algorithm and store results."""
        fn = ALGORITHMS[self.algo_name]
        self.explore_order, self.parent_map = fn(
            self.grid, self.start_pos, self.goal_pos,
            GRID_ROWS, GRID_COLS)
        self.final_path = reconstruct_path(
            self.parent_map, self.start_pos, self.goal_pos)

    def start_simulation(self):
        if self.state == SIM_RUNNING:
            self.state = SIM_PAUSED
            return
        if self.state == SIM_PAUSED:
            self.state = SIM_RUNNING
            return

        # Fresh start
        self.run_algorithm()
        self.explore_idx  = 0
        self.path_idx     = 0
        self.robot_pos    = self.start_pos
        self.phase        = "explore"
        self.step_timer   = 0.0

        if not self.explore_order:
            self.state = SIM_NO_PATH
        else:
            self.state = SIM_RUNNING

    def reset_simulation(self):
        """Reset animation but keep grid and obstacles."""
        self.explore_idx  = 0
        self.path_idx     = 0
        self.robot_pos    = self.start_pos
        self.phase        = "explore"
        self.explore_order = []
        self.parent_map   = {}
        self.final_path   = []
        self.state        = SIM_IDLE
        self.step_timer   = 0.0

    def clear_obstacles(self):
        """Remove all obstacles, keep start and goal."""
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if self.grid[r][c] == OBSTACLE:
                    self.grid[r][c] = EMPTY
        self.reset_simulation()

    def toggle_obstacle(self, r, c):
        """Toggle obstacle on a cell (respects start/goal)."""
        if (r, c) in (self.start_pos, self.goal_pos):
            return
        if self.grid[r][c] == OBSTACLE:
            self.grid[r][c] = EMPTY
        else:
            self.grid[r][c] = OBSTACLE
        self.reset_simulation()

    def set_start(self, r, c):
        if self.grid[r][c] == GOAL:
            return
        # Clear old start
        sr, sc = self.start_pos
        self.grid[sr][sc] = EMPTY
        self.start_pos = (r, c)
        self.grid[r][c] = START
        self.reset_simulation()

    def set_goal(self, r, c):
        if self.grid[r][c] == START:
            return
        gr, gc = self.goal_pos
        self.grid[gr][gc] = EMPTY
        self.goal_pos = (r, c)
        self.grid[r][c] = GOAL
        self.reset_simulation()

    def switch_algo(self, idx):
        self.algo_idx  = idx
        self.algo_name = ALGO_NAMES[idx]
        self.reset_simulation()

    # ── Update (called each frame) ────────────

    def update(self, dt):
        if self.state != SIM_RUNNING:
            return

        step_time = 1.0 / SPEEDS[self.speed_idx]
        self.step_timer += dt

        while self.step_timer >= step_time:
            self.step_timer -= step_time
            self._advance_step()
            if self.state != SIM_RUNNING:
                break

    def _advance_step(self):
        """Advance animation by one step."""
        if self.phase == "explore":
            if self.explore_idx < len(self.explore_order):
                self.explore_idx += 1
            else:
                # Exploration done — switch to path phase
                if self.final_path:
                    self.phase = "path"
                    self.path_idx = 0
                else:
                    self.state = SIM_NO_PATH

        elif self.phase == "path":
            if self.path_idx < len(self.final_path):
                self.robot_pos = self.final_path[self.path_idx]
                self.path_idx  += 1
            else:
                self.state = SIM_DONE
                self.phase = "done"

    # ── Draw sets for renderer ────────────────

    @property
    def explored_set(self):
        return set(self.explore_order[:self.explore_idx])

    @property
    def path_set(self):
        if self.phase in ("path", "done"):
            return set(self.final_path[:self.path_idx])
        return set()


# ─────────────────────────────────────────────
#  MAIN ENTRY POINT
# ─────────────────────────────────────────────

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Robot Navigation Simulator — AI Search Algorithms")

    clock  = pygame.time.Clock()
    fonts  = load_fonts()
    sim    = RobotSimulator()

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0   # delta time in seconds
        sim.tick += 1

        # ── EVENT HANDLING ──────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_SPACE:
                    sim.start_simulation()

                elif event.key == pygame.K_r:
                    sim.reset_simulation()

                elif event.key == pygame.K_c:
                    sim.clear_obstacles()

                elif event.key == pygame.K_1:
                    sim.switch_algo(0)
                elif event.key == pygame.K_2:
                    sim.switch_algo(1)
                elif event.key == pygame.K_3:
                    sim.switch_algo(2)

                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    sim.speed_idx = min(sim.speed_idx + 1, len(SPEEDS) - 1)
                elif event.key in (pygame.K_MINUS, pygame.K_UNDERSCORE, pygame.K_KP_MINUS):
                    sim.speed_idx = max(sim.speed_idx - 1, 0)

                elif event.key == pygame.K_s:
                    sim.place_mode = "S" if sim.place_mode != "S" else None
                elif event.key == pygame.K_g:
                    sim.place_mode = "G" if sim.place_mode != "G" else None

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mx, my = event.pos
                    if mx < GRID_COLS * CELL_SIZE:
                        r, c = my // CELL_SIZE, mx // CELL_SIZE
                        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                            if sim.place_mode == "S":
                                sim.set_start(r, c)
                                sim.place_mode = None
                            elif sim.place_mode == "G":
                                sim.set_goal(r, c)
                                sim.place_mode = None
                            else:
                                sim.toggle_obstacle(r, c)
                                sim.mouse_held = True

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    sim.mouse_held = False

            elif event.type == pygame.MOUSEMOTION:
                if sim.mouse_held and sim.place_mode is None:
                    mx, my = event.pos
                    if mx < GRID_COLS * CELL_SIZE:
                        r, c = my // CELL_SIZE, mx // CELL_SIZE
                        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                            if (r, c) not in (sim.start_pos, sim.goal_pos):
                                sim.grid[r][c] = OBSTACLE
                                sim.reset_simulation()

        # ── UPDATE ──────────────────────────────
        sim.update(dt)

        # ── DRAW ────────────────────────────────
        draw_grid(
            screen,
            sim.grid, GRID_ROWS, GRID_COLS,
            sim.explored_set,
            sim.path_set,
            sim.robot_pos,
            sim.goal_pos,
            sim.start_pos,
            sim.tick,
        )

        draw_panel(
            screen,
            sim.state,
            sim.algo_name,
            sim.steps,
            sim.path_len if sim.state in (SIM_DONE, SIM_RUNNING, SIM_PAUSED) else 0,
            sim.explored_count,
            sim.speed_idx,
            sim.place_mode,
            fonts,
        )

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
