"""
visualize.py — Interactive A* Pathfinding Visualizer
"""

from __future__ import annotations
import pygame
import json
import random
import heapq
import time
import tkinter as tk
from tkinter import filedialog
from typing import Iterator, List, Optional, Tuple

# ─── Constants ────────────────────────────────────────────────────────────────

ROWS = 101
START = (0, 0)
GOAL = (ROWS - 1, ROWS - 1)

# ─── Maze Generator (from create_grid_worlds.py) ──────────────────────────────


def create_maze(rows: int = ROWS) -> List[List[int]]:
    maze = [[0] * rows for _ in range(rows)]
    tracker = [[0] * rows for _ in range(rows)]
    stack = []
    r, c = 0, 0
    tracker[0][0] = 1

    while any(0 in row for row in tracker):
        neighbors = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
        random.shuffle(neighbors)
        found = False
        for nr, nc in neighbors:
            if 0 <= nr < rows and 0 <= nc < rows and tracker[nr][nc] == 0:
                tracker[nr][nc] = 1
                if random.random() < 0.3:
                    maze[nr][nc] = 1
                else:
                    stack.append((r, c))
                    r, c = nr, nc
                found = True
                break
        if not found:
            if stack:
                r, c = stack.pop()
            else:
                jumped = False
                for i in range(rows):
                    for j in range(rows):
                        if tracker[i][j] == 0:
                            tracker[i][j] = 1
                            maze[i][j] = 0
                            r, c = i, j
                            jumped = True
                            break
                    if jumped:
                        break

    maze[0][0] = 0
    maze[rows - 1][rows - 1] = 0
    return maze


# ─── Priority Queues (from custom_pq.py) ──────────────────────────────────────


class CustomPQ_maxG:
    def __init__(self):
        self._h = []
        self._c = 0

    def push(self, f, g, cell):
        heapq.heappush(self._h, (f, -g, self._c, cell))
        self._c += 1

    def pop(self):
        return heapq.heappop(self._h)[3]

    def is_empty(self):
        return len(self._h) == 0


class CustomPQ_minG:
    def __init__(self):
        self._h = []
        self._c = 0

    def push(self, f, g, cell):
        heapq.heappush(self._h, (f, g, self._c, cell))
        self._c += 1

    def pop(self):
        return heapq.heappop(self._h)[3]

    def is_empty(self):
        return len(self._h) == 0


# ─── Forward A* (from q2.py) ──────────────────────────────────────────────────


def fwd_astar_gen(
    actual_maze: List[List[int]],
    start: Tuple[int, int] = START,
    goal: Tuple[int, int] = GOAL,
    tie_breaking: str = "max_g",
) -> Iterator[dict]:

    expanded_all: set = set()
    expanded_cur: set = set()

    def compute_path(start, goal, known_map):
        closed = set()
        g = {start: 0}
        tree = {}
        pq = CustomPQ_maxG() if tie_breaking == "max_g" else CustomPQ_minG()

        def h(cell):
            return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

        pq.push(g[start] + h(start), g[start], start)
        current = start

        while not pq.is_empty():
            current = pq.pop()
            if current == goal:
                break
            if current in closed:
                continue
            closed.add(current)
            expanded_all.add(current)
            expanded_cur.add(current)

            # yield after every expansion so pygame can draw it
            yield {
                "phase": "search",
               "expanded_old": (expanded_all - expanded_cur).copy(),  # previous replans only
                "expanded_cur": expanded_cur.copy(),                    # current replan
                "path": [],  # no planned path yet during search
                "executed": executed_path.copy(),  
                "agent": start,  # keep showing agent at current pos
                "done": False,
                "found": False,
                "replans": total_replans,  # ← pass actual replans count
                "total_exp": len(expanded_all),
            }

            r, c = current
            for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= nr < ROWS and 0 <= nc < ROWS and known_map[nr][nc] == 0:
                    new_g = g[current] + 1
                    if new_g < g.get((nr, nc), float("inf")):
                        g[(nr, nc)] = new_g
                        tree[(nr, nc)] = current
                        pq.push(new_g + h((nr, nc)), new_g, (nr, nc))

        if current != goal:
            return None
        path = []
        node = goal
        while node != start:
            path.append(node)
            node = tree[node]
        path.append(start)
        path.reverse()
        return path

    known_map = [[0] * ROWS for _ in range(ROWS)]
    current_position = start
    executed_path = [start]
    total_replans = 0

    while current_position != goal:
        r, c = current_position
        for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
            if 0 <= nr < ROWS and 0 <= nc < ROWS:
                known_map[nr][nc] = actual_maze[nr][nc]
        expanded_cur.clear()
        path = yield from compute_path(current_position, goal, known_map)

        if path is None:
            yield {
                "expanded_old": (expanded_all - expanded_cur).copy(),
                "expanded_cur": expanded_cur.copy(),
                "path": [],
                "executed": executed_path.copy(),
                "agent": current_position,
                "done": True,
                "found": False,
                "replans": total_replans,
                "total_exp": len(expanded_all),
                "phase": "done",
            }
            return
        total_replans += 1

        for next_cell in path[1:]:
            r, c = next_cell
            for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= nr < ROWS and 0 <= nc < ROWS:
                    known_map[nr][nc] = actual_maze[nr][nc]
            if known_map[next_cell[0]][next_cell[1]] == 1:
                break
            current_position = next_cell
            executed_path.append(current_position)

            yield {
                "expanded_old": expanded_all.copy(),
                "expanded_cur": set(),
                "phase": "move",
                "path": path,
                "executed": executed_path.copy(),
                "agent": current_position,
                "done": current_position == goal,
                "found": current_position == goal,
                "replans": total_replans,
                "total_exp": len(expanded_all),
            }
            if current_position == goal:
                return


# ─── Backward A* (from q3.py) ─────────────────────────────────────────────────


def bwd_astar_gen(
    actual_maze: List[List[int]],
    start: Tuple[int, int] = START,
    goal: Tuple[int, int] = GOAL,
) -> Iterator[dict]:

    expanded_all: set = set()
    expanded_cur: set = set()

    def compute_path(search_start, search_goal, known_map):
        closed = set()
        g = {search_start: 0}
        tree = {}
        pq = CustomPQ_maxG()

        def h(cell, target=search_goal):
            return abs(cell[0] - target[0]) + abs(cell[1] - target[1])

        pq.push(g[search_start] + h(search_start), g[search_start], search_start)
        current = search_start

        while not pq.is_empty():
            current = pq.pop()
            if current == search_goal:
                break
            if current in closed:
                continue
            closed.add(current)
            expanded_all.add(current)
            expanded_cur.add(current)

            yield {
                "phase": "search",
                "expanded_old": (expanded_all - expanded_cur).copy(),  # previous replans only
                "expanded_cur": expanded_cur.copy(),                    # current replan
                "path": [],  # no planned path yet during search
                "executed": executed_path.copy(), 
                "agent": start,  # keep showing agent at current pos
                "done": False,
                "found": False,
                "replans": total_replans,  # ← pass actual replans count
                "total_exp": len(expanded_all),
            }

            r, c = current
            for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= nr < ROWS and 0 <= nc < ROWS and known_map[nr][nc] == 0:
                    new_g = g[current] + 1
                    if new_g < g.get((nr, nc), float("inf")):
                        g[(nr, nc)] = new_g
                        tree[(nr, nc)] = current
                        pq.push(new_g + h((nr, nc)), new_g, (nr, nc))

        if current != search_goal:
            return None
        path = []
        node = search_goal
        while node != search_start:
            path.append(node)
            node = tree[node]
        path.append(search_start)
        path.reverse()
        return path

    known_map = [[0] * ROWS for _ in range(ROWS)]
    current_position = start
    executed_path = [start]
    total_replans = 0

    while current_position != goal:
        r, c = current_position
        for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
            if 0 <= nr < ROWS and 0 <= nc < ROWS:
                known_map[nr][nc] = actual_maze[nr][nc]

        expanded_cur.clear()
        path = yield from compute_path(goal, current_position, known_map)

        if path is None:
            yield {
                "expanded_old": (expanded_all - expanded_cur).copy(),
                "expanded_cur": expanded_cur.copy(),
                "path": [],
                "executed": executed_path.copy(),
                "agent": current_position,
                "done": True,
                "found": False,
                "replans": total_replans,
                "total_exp": len(expanded_all),
                "phase": "done",
            }
            return
        path = path[::-1]
        total_replans += 1

        for next_cell in path[1:]:
            r, c = next_cell
            for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= nr < ROWS and 0 <= nc < ROWS:
                    known_map[nr][nc] = actual_maze[nr][nc]
            if known_map[next_cell[0]][next_cell[1]] == 1:
                break
            current_position = next_cell
            executed_path.append(current_position)

            yield {
                "expanded_old": expanded_all.copy(),
                "expanded_cur": set(),
                "phase": "move",
                "path": path,
                "executed": executed_path.copy(),
                "agent": current_position,
                "done": current_position == goal,
                "found": current_position == goal,
                "replans": total_replans,
                "total_exp": len(expanded_all),
            }
            if current_position == goal:
                return


# ─── Adaptive A* (from q5.py) ─────────────────────────────────────────────────


def adaptive_astar_gen(
    actual_maze: List[List[int]],
    start: Tuple[int, int] = START,
    goal: Tuple[int, int] = GOAL,
) -> Iterator[dict]:

    expanded_all: set = set()
    expanded_cur: set = set()
    h_vals = {}

    def compute_path(start, goal, known_map):
        closed = set()
        g = {start: 0}
        tree = {}
        pq = CustomPQ_maxG()

        def h(cell):
            if cell in h_vals:
                return h_vals[cell]
            return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

        pq.push(g[start] + h(start), g[start], start)
        current = start

        while not pq.is_empty():
            current = pq.pop()
            if current == goal:
                break
            if current in closed:
                continue
            closed.add(current)
            expanded_all.add(current)
            expanded_cur.add(current)

            yield {
                "phase": "search",
                "expanded_old": (expanded_all - expanded_cur).copy(),  # previous replans only
                "expanded_cur": expanded_cur.copy(),                    # current replan
                "path": [],  # no planned path yet during search
                "executed": executed_path.copy(),  
                "agent": start,  # keep showing agent at current pos
                "done": False,
                "found": False,
                "replans": total_replans,  # ← pass actual replans count
                "total_exp": len(expanded_all),
            }

            r, c = current
            for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= nr < ROWS and 0 <= nc < ROWS and known_map[nr][nc] == 0:
                    new_g = g[current] + 1
                    if new_g < g.get((nr, nc), float("inf")):
                        g[(nr, nc)] = new_g
                        tree[(nr, nc)] = current
                        pq.push(new_g + h((nr, nc)), new_g, (nr, nc))

        if current != goal:
            return None, g, closed
        path = []
        node = goal
        while node != start:
            path.append(node)
            node = tree[node]
        path.append(start)
        path.reverse()
        return path, g, closed

    known_map = [[0] * ROWS for _ in range(ROWS)]
    current_position = start
    executed_path = [start]
    total_replans = 0

    while current_position != goal:
        r, c = current_position
        for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
            if 0 <= nr < ROWS and 0 <= nc < ROWS:
                known_map[nr][nc] = actual_maze[nr][nc]

        expanded_cur.clear()
        result = yield from compute_path(current_position, goal, known_map)

        if result is None or result[0] is None:
            yield {
                "expanded_old": (expanded_all - expanded_cur).copy(),
                "expanded_cur": expanded_cur.copy(),
                "path": [],
                "executed": executed_path.copy(),
                "agent": current_position,
                "done": True,
                "found": False,
                "replans": total_replans,
                "total_exp": len(expanded_all),
            }
            return

        path, g, closed_set = result
        g_goal = g.get(goal, float("inf"))
        for cell in closed_set:
            h_vals[cell] = g_goal - g.get(cell, 0)
        total_replans += 1

        for next_cell in path[1:]:
            r, c = next_cell
            for nr, nc in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= nr < ROWS and 0 <= nc < ROWS:
                    known_map[nr][nc] = actual_maze[nr][nc]
            if known_map[next_cell[0]][next_cell[1]] == 1:
                break
            current_position = next_cell
            executed_path.append(current_position)

            yield {
                "expanded_old": expanded_all.copy(),
                "expanded_cur": set(),
                "phase": "move",
                "path": path,
                "executed": executed_path.copy(),
                "agent": current_position,
                "done": current_position == goal,
                "found": current_position == goal,
                "replans": total_replans,
                "total_exp": len(expanded_all),
            }
            if current_position == goal:
                return


# ─── Window / Display Constants ──────────────────────

PANEL_W = 260  # sidebar width in pixels, fixed
MIN_CELL = 4  # minimum cell size before grid gets unreadable


# ─── Colors ─────────────────────────────────────────

C_BG = (45, 45, 45)  # dark grey — background and gaps between cells
C_FREE = (80, 80, 80)  # medium grey — visible against bg but not loud
C_BLOCKED = (255, 255, 255)  # white — stands out clearly
C_PATH = (128, 0, 0)  # maroon — executed path
C_PLANNED = (200, 80, 80)  # lighter maroon — planned but not yet walked
C_AGENT = (255, 220, 50)  # yellow — agent position
C_START = (50, 205, 50)  # green — start
C_GOAL = (180, 80, 220)  # purple — goal
C_TEXT = (220, 220, 220)
C_SUBTEXT = (140, 140, 140)
C_BTN = (60, 60, 80)
C_BTN_HOV = (80, 80, 110)
C_BTN_ACT = (80, 120, 200)
C_ACCENT = (80, 120, 200)

C_EXPANDED_OLD = (50, 80, 140)  # darker blue — previous replans
C_EXPANDED_CUR = (
    100,
    149,
    237,
)  # bright blue — current replan 

FPS = 60


def get_cell_size(win_h: int) -> int:
    """Compute cell size so the grid fills the window height."""
    return max(MIN_CELL, win_h // ROWS)


# ─── draw_grid() —───────────────────────────────────────────────────────


def draw_grid(surf, maze, state: Optional[dict], ox=0, oy=0):
    cell_size = get_cell_size(surf.get_height())
    rows = len(maze)

    expanded_old = state.get("expanded_old", set()) if state else set()
    expanded_cur = state.get("expanded_cur", set()) if state else set()
    path_set = set(state["path"]) if state else set()
    executed = set(state["executed"]) if state else set()
    agent = state["agent"] if state else START

    for r in range(rows):
        for c in range(rows):
            x = ox + c * cell_size
            y = oy + r * cell_size
            cell = (r, c)

            if maze[r][c] == 1:
                color = C_BLOCKED
            elif cell == agent:
                color = C_AGENT
            elif cell == GOAL:
                color = C_GOAL
            elif cell == START:
                color = C_START
            elif cell in executed:
                color = C_PATH
            elif cell in path_set:
                color = C_PLANNED
            elif cell in expanded_cur:
                color = C_EXPANDED_CUR
            elif cell in expanded_old:
                color = C_EXPANDED_OLD
            else:
                color = C_FREE

            pygame.draw.rect(surf, color, (x, y, cell_size - 1, cell_size - 1))


# ─── draw_text() —───────────────────────────────────────────────────────


def file_dialog_open():
    root = tk.Tk()
    root.withdraw()
    path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
    root.destroy()
    return path or None


def file_dialog_save():
    root = tk.Tk()
    root.withdraw()
    path = filedialog.asksaveasfilename(
        defaultextension=".json", filetypes=[("JSON", "*.json")]
    )
    root.destroy()
    return path or None


def draw_text(surf, text, x, y, color=C_TEXT, size=16, bold=False):
    font = pygame.font.SysFont("monospace", size, bold=bold)
    surf.blit(font.render(text, True, color), (x, y))


# ─── draw_button() —─────────────────────────────────────────────────────


def draw_button(surf, rect, label, hovered=False, active=False, size=15):
    color = C_BTN_ACT if active else (C_BTN_HOV if hovered else C_BTN)
    pygame.draw.rect(surf, color, rect, border_radius=6)
    pygame.draw.rect(surf, C_ACCENT, rect, 1, border_radius=6)
    font = pygame.font.SysFont("monospace", size, bold=True)
    text = font.render(label, True, C_TEXT)
    surf.blit(text, text.get_rect(center=rect.center))


# ─── setup_screen() —────────────────────────────────────────────────────
# Shows: generate / reuse / import buttons, algo selector, run button
# Returns: (maze, algo_name) or None


def setup_screen(win, clock, last_maze):
    algos = ["Forward A*", "Backward A*", "Adaptive A*"]
    algo_keys = ["fwd", "bwd", "adaptive"]
    selected = 0  # index of currently selected algorithm
    maze = last_maze  # may be None if first run
    status = ""  # message shown below buttons

    while True:
        # ── recompute layout every frame from current window size ──
        W, H = win.get_size()
        cx = W // 2
        mx, my = pygame.mouse.get_pos()

        btn_w, btn_h = 220, 40
        bx = cx - btn_w // 2  # left edge of buttons, centered horizontally

        # maze buttons
        btn_new = pygame.Rect(bx, int(H * 0.16), btn_w, btn_h)
        btn_reuse = pygame.Rect(bx, int(H * 0.24), btn_w, btn_h)
        btn_import = pygame.Rect(bx, int(H * 0.32), btn_w, btn_h)

        # algorithm selector buttons
        btn_algo = [
            pygame.Rect(bx, int(H * 0.48) + i * 50, btn_w, btn_h) for i in range(3)
        ]

        # run button near bottom
        btn_run = pygame.Rect(bx, int(H * 0.82), btn_w, 46)

        # ── draw ──
        win.fill(C_BG)

        draw_text(
            win,
            "A* Pathfinding Visualizer",
            cx - 160,
            int(H * 0.05),
            C_TEXT,
            size=22,
            bold=True,
        )

        draw_text(win, "Maze", bx, int(H * 0.12), C_SUBTEXT, size=13)
        draw_button(win, btn_new, "Generate New Maze", btn_new.collidepoint(mx, my))
        draw_button(win, btn_reuse, "Reuse Last Maze", btn_reuse.collidepoint(mx, my))
        draw_button(win, btn_import, "Import JSON", btn_import.collidepoint(mx, my))

        draw_text(win, "Algorithm", bx, int(H * 0.43), C_SUBTEXT, size=13)
        for i, label in enumerate(algos):
            draw_button(
                win,
                btn_algo[i],
                label,
                btn_algo[i].collidepoint(mx, my),
                active=(i == selected),
            )

        draw_button(
            win, btn_run, "▶  RUN", btn_run.collidepoint(mx, my), active=True, size=17
        )

        if status:
            draw_text(win, status, bx, int(H * 0.75), C_SUBTEXT, size=13)

        pygame.display.flip()
        clock.tick(FPS)

        # ── events ──
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return None
            if event.type == pygame.VIDEORESIZE:
                win = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            if event.type == pygame.MOUSEBUTTONDOWN:
                if btn_new.collidepoint(mx, my):
                    maze = create_maze()
                    status = "New maze generated."
                elif btn_reuse.collidepoint(mx, my):
                    if maze is not None:
                        status = "Reusing last maze."
                    else:
                        status = "No maze yet — generate one first."
                elif btn_import.collidepoint(mx, my):
                    path = file_dialog_open()
                    if path:
                        try:
                            with open(path) as f:
                                data = json.load(f)
                            # handle both single maze and list-of-mazes JSON
                            if (
                                isinstance(data, list)
                                and isinstance(data[0], list)
                                and isinstance(data[0][0], list)
                            ):
                                maze = data[0]
                            else:
                                maze = data
                            maze[0][0] = 0
                            maze[-1][-1] = 0
                            status = f"Imported: {path.split('/')[-1]}"
                        except Exception as e:
                            status = f"Import failed: {e}"
                for i in range(3):
                    if btn_algo[i].collidepoint(mx, my):
                        selected = i
                if btn_run.collidepoint(mx, my):
                    if maze is None:
                        status = "Generate or import a maze first."
                    else:
                        return maze, algo_keys[selected]


# ─── vis_screen() —──────────────────────────────────────────────────────
# Shows: grid animation, stats panel, pause/back/export buttons
# Returns: maze (for potential reuse)


def vis_screen(win, clock, maze, algo_name):
    # ── setup ──
    fast_mode = False
    speed = 1  # steps advanced per frame
    paused = False
    state = None  # current algorithm state, None until first step
    done = False
    elapsed = 0.0
    start_time = time.time()
    status = ""

    # build the generator based on chosen algorithm
    if algo_name == "fwd":
        gen = fwd_astar_gen(maze)
        title = "Repeated Forward A*"
    elif algo_name == "bwd":
        gen = bwd_astar_gen(maze)
        title = "Repeated Backward A*"
    else:
        gen = adaptive_astar_gen(maze)
        title = "Adaptive A*"

    while True:
        W, H = win.get_size()
        mx, my = pygame.mouse.get_pos()

        # derive grid dimensions from window height each frame
        cell_size = get_cell_size(H)
        grid_px = ROWS * cell_size
        px = grid_px + 10  # left edge of panel

        # panel buttons — anchored to bottom of window
        btn_pause = pygame.Rect(px, H - 160, PANEL_W - 20, 40)
        btn_export = pygame.Rect(px, H - 110, PANEL_W - 20, 40)
        btn_back = pygame.Rect(px, H - 60, PANEL_W - 20, 40)
        btn_speed = pygame.Rect(px, H - 210, PANEL_W - 20, 40)

        # ── advance generator ──
        if not paused and not done:
            steps = 50 if fast_mode else speed
            for _ in range(steps):
                try:
                    state = next(gen)
                    if state.get("phase") == "move" and not fast_mode:
                        pygame.time.delay(80)
                    if state["done"]:
                        done = True
                        elapsed = time.time() - start_time
                        break
                except StopIteration:
                    done = True
                    elapsed = time.time() - start_time
                    break

        # ── draw ──
        win.fill(C_BG)
        draw_grid(win, maze, state)
        pygame.draw.rect(win, C_ACCENT, (0, 0, grid_px, H), 1)  # grid border
        pygame.draw.line(win, C_ACCENT, (grid_px, 0), (grid_px, H), 1)  # panel divider

        # ── panel ──
        y = 20
        draw_text(win, title, px, y, C_TEXT, size=14, bold=True)
        y += 30
        draw_text(win, f"Grid: {ROWS}x{ROWS}", px, y, C_SUBTEXT, size=13)
        y += 20
        draw_text(win, f"Speed: {speed}x", px, y, C_SUBTEXT, size=12)
        y += 10
        draw_text(win, "── Controls ──", px, y, C_SUBTEXT, size=12)
        y += 18
        draw_text(win, "↑↓  speed", px, y, C_TEXT, size=12)
        y += 16
        draw_text(win, "SPACE  pause", px, y, C_TEXT, size=12)
        y += 16
        draw_text(win, "ESC  back", px, y, C_TEXT, size=12)
        y += 20

        # stats
        if state:
            draw_text(win, "── Stats ──", px, y, C_SUBTEXT, size=12)
            y += 20
            draw_text(win, f"Expanded : {state['total_exp']}", px, y, C_TEXT, size=13)
            y += 18
            draw_text(win, f"Replans  : {state['replans']}", px, y, C_TEXT, size=13)
            y += 18
            draw_text(
                win, f"Walked   : {len(state['executed'])}", px, y, C_TEXT, size=13
            )
            y += 24

        if done and state:
            result_text = "PATH FOUND ✓" if state["found"] else "NO PATH ✗"
            result_color = (80, 220, 80) if state["found"] else (220, 80, 80)
            draw_text(win, result_text, px, y, result_color, size=14, bold=True)
            y += 24
            draw_text(win, f"Time: {elapsed:.2f}s", px, y, C_SUBTEXT, size=13)
            y += 20

        # legend — pinned to middle of panel
        y = H // 2
        draw_text(win, "── Legend ──", px, y, C_SUBTEXT, size=12)
        y += 18
        for label, color in [
            ("Start", C_START),
            ("Goal", C_GOAL),
            ("Agent", C_AGENT),
            ("Searching", C_EXPANDED_CUR),
            ("Explored",  C_EXPANDED_OLD),
            ("Planned", C_PLANNED),
            ("Walked", C_PATH),
            ("Blocked", C_BLOCKED),
            ("Free", C_FREE),
        ]:
            pygame.draw.rect(win, color, (px, y + 1, 14, 14))
            draw_text(win, label, px + 20, y, C_TEXT, size=12)
            y += 18

        # buttons
        draw_button(
            win,
            btn_pause,
            "|| Pause" if not paused else ">> Resume",
            btn_pause.collidepoint(mx, my),
        )
        draw_button(
            win, btn_export, "Export Maze JSON", btn_export.collidepoint(mx, my)
        )
        draw_button(win, btn_back, "<-- Back", btn_back.collidepoint(mx, my))

        draw_button(win, btn_speed,
            ">> Fast" if not fast_mode else "|| Normal",
            btn_speed.collidepoint(mx, my),
            active=fast_mode)

        if status:
            draw_text(win, status, px, H - 185, C_SUBTEXT, size=11)

        pygame.display.flip()
        clock.tick(FPS)

        # ── events ──
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            if event.type == pygame.VIDEORESIZE:
                win = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return maze
                if event.key == pygame.K_UP:
                    speed = min(speed + 1, 30)
                if event.key == pygame.K_DOWN:
                    speed = max(speed - 1, 1)
                if event.key == pygame.K_SPACE:
                    paused = not paused
            if event.type == pygame.MOUSEBUTTONDOWN:
                if btn_back.collidepoint(mx, my):
                    return maze
                if btn_speed.collidepoint(mx, my):
                    fast_mode = not fast_mode
                if btn_pause.collidepoint(mx, my):
                    paused = not paused
                if btn_export.collidepoint(mx, my):
                    path = file_dialog_save()
                    if path:
                        with open(path, "w") as f:
                            json.dump(maze, f)
                        status = f"Saved: {path.split('/')[-1]}"


# ─── main() —────────────────────────────────────────────────────────────


def main():
    pygame.init()
    win = pygame.display.set_mode((1100, 900), pygame.RESIZABLE)
    pygame.display.set_caption("A* Pathfinding Visualizer")
    clock = pygame.time.Clock()

    last_maze = None

    while True:
        result = setup_screen(win, clock, last_maze)
        if result is None:
            break
        maze, algo = result
        last_maze = maze
        returned = vis_screen(win, clock, maze, algo)
        if returned is None:
            break
        last_maze = returned

    pygame.quit()


if __name__ == "__main__":
    main()
