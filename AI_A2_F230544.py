import pygame, heapq, random, time, math

# ── Config ────────────────────────────────────────────
WIDTH, HEIGHT = 700, 560
COLS,  ROWS   = 20, 20
CELL          = 22
GRID_X        = (WIDTH - COLS * CELL) // 2   # centered horizontally
GRID_Y        = 90

# ── Colors ────────────────────────────────────────────
DARKBG    = (22,  25,  40)
TOOLBAR   = (15,  18,  30)
EMPTY     = (30,  34,  55)
WALL      = (70,  75, 115)
START_C   = (55, 205,  90)
GOAL_C    = (215,  55,  55)
VISITED_C = (45,  88, 165)
FRONTIER_C= (80, 150, 230)
PATH_C    = (255, 205,  45)
WHITE     = (255, 255, 255)
BLACK     = (0,   0,   0)
DIM       = (110, 120, 158)
YELLOW    = (255, 215,  50)
GREEN     = (55,  205,  85)
RED       = (215,  55,  55)
GRIDLINE  = (35,  39,  62)

# ── Heuristics ────────────────────────────────────────
def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# ── Search helpers ────────────────────────────────────
def get_neighbors(node, walls):
    c, r = node
    for dc, dr in [(0,1),(0,-1),(1,0),(-1,0)]:
        nb = (c+dc, r+dr)
        if 0 <= nb[0] < COLS and 0 <= nb[1] < ROWS and nb not in walls:
            yield nb

def reconstruct(came_from, goal):
    path, node = [], goal
    while node in came_from:
        path.append(node); node = came_from[node]
    path.append(node); path.reverse()
    return path

# ── Algorithms (generators) ───────────────────────────
def run_astar(start, goal, walls, h):
    oq = [(h(start,goal), 0, start)]
    cf = {start: None}; g = {start: 0}
    exp = set(); os = {start}
    while oq:
        _, gc, cur = heapq.heappop(oq)
        os.discard(cur)
        if cur == goal:
            yield exp, os, reconstruct(cf, goal); return
        if cur in exp: continue
        exp.add(cur)
        for nb in get_neighbors(cur, walls):
            ng = gc + 1
            if nb not in g or ng < g[nb]:
                g[nb] = ng; cf[nb] = cur
                heapq.heappush(oq, (ng + h(nb,goal), ng, nb)); os.add(nb)
        yield exp, os, []
    yield exp, os, []

def run_gbfs(start, goal, walls, h):
    oq = [(h(start,goal), start)]
    cf = {start: None}; vis = {start}; os = {start}
    while oq:
        _, cur = heapq.heappop(oq)
        os.discard(cur)
        if cur == goal:
            yield vis, os, reconstruct(cf, goal); return
        for nb in get_neighbors(cur, walls):
            if nb not in vis:
                vis.add(nb); cf[nb] = cur
                heapq.heappush(oq, (h(nb,goal), nb)); os.add(nb)
        yield vis, os, []
    yield vis, os, []

def run_ucs(start, goal, walls, h):
    oq = [(0, start)]
    cf = {start: None}; g = {start: 0}
    exp = set(); os = {start}
    while oq:
        gc, cur = heapq.heappop(oq)
        os.discard(cur)
        if cur == goal:
            yield exp, os, reconstruct(cf, goal); return
        if cur in exp: continue
        exp.add(cur)
        for nb in get_neighbors(cur, walls):
            ng = gc + 1
            if nb not in g or ng < g[nb]:
                g[nb] = ng; cf[nb] = cur
                heapq.heappush(oq, (ng, nb)); os.add(nb)
        yield exp, os, []
    yield exp, os, []

# ── Helpers ───────────────────────────────────────────
def cell_px(c, r): return GRID_X + c*CELL, GRID_Y + r*CELL
def px_cell(x, y): return (x-GRID_X)//CELL, (y-GRID_Y)//CELL
def in_grid(x, y):
    return GRID_X<=x<GRID_X+COLS*CELL and GRID_Y<=y<GRID_Y+ROWS*CELL

# ── Button ────────────────────────────────────────────
class Btn:
    def __init__(self, x, y, w, h, label):
        self.rect  = pygame.Rect(x, y, w, h)
        self.label = label
    def draw(self, surf, font, active=False):
        bg = (60, 110, 195) if active else (38, 44, 72)
        pygame.draw.rect(surf, bg,           self.rect, border_radius=5)
        pygame.draw.rect(surf, (80, 95, 145), self.rect, 1, border_radius=5)
        t = font.render(self.label, True, WHITE)
        surf.blit(t, t.get_rect(center=self.rect.center))
    def hit(self, pos): return self.rect.collidepoint(pos)

# ── Main ──────────────────────────────────────────────
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Dynamic Pathfinding Agent")
    clock = pygame.time.Clock()
    fsm = pygame.font.SysFont("consolas", 13)
    fmd = pygame.font.SysFont("consolas", 14, bold=True)

    START = (0, 10); GOAL = (19, 10)

    # Dynamic walls — regenerated every interval
    walls = set()
    DYN_INTERVAL = 80   # frames between wall changes

    def new_map():
        walls.clear()
        for _ in range(55):   # ~14% density for 20x20
            c = random.randint(0, COLS-1); r = random.randint(0, ROWS-1)
            if (c,r) != START and (c,r) != GOAL:
                walls.add((c,r))

    new_map()

    # Search state
    algo  = "astar"; hfunc = manhattan; hname = "Manhattan"
    gen   = None
    visited = set(); frontier = set(); path = []
    searching = False; done = False
    nodes_exp = 0; path_cost = 0; elapsed = 0.0; t0 = 0.0
    dyn_mode  = False; dyn_timer = 0; replans = 0
    DELAY = 0.02; last_step = 0.0

    def start_search():
        nonlocal gen, visited, frontier, path, searching, done
        nonlocal nodes_exp, path_cost, elapsed, t0
        walls.discard(START); walls.discard(GOAL)
        visited=set(); frontier=set(); path=[]; done=False
        nodes_exp=path_cost=0; elapsed=0.0; t0=time.time()
        if   algo=="astar": gen = run_astar(START, GOAL, walls, hfunc)
        elif algo=="gbfs":  gen = run_gbfs (START, GOAL, walls, hfunc)
        else:               gen = run_ucs  (START, GOAL, walls, hfunc)
        searching = True

    def clear_search():
        nonlocal gen, visited, frontier, path, searching, done, nodes_exp, path_cost, elapsed
        gen=None; visited=set(); frontier=set(); path=[]
        searching=done=False; nodes_exp=path_cost=0; elapsed=0.0

    def step_search():
        nonlocal visited, frontier, path, searching, done, nodes_exp, path_cost, elapsed
        if not gen: return
        try:
            v, f, p = next(gen)
            visited, frontier = v, f; nodes_exp = len(v)
            if p:
                path=p; path_cost=len(p)-1; elapsed=time.time()-t0
                searching=False; done=True
        except StopIteration:
            searching=False; done=True; elapsed=time.time()-t0

    # ── Layout: center buttons above grid ────────────
    cx = WIDTH // 2

    # Row 1 — algo + run + clear
    bw1, bh, gap = 72, 26, 6
    total1 = 3*bw1 + 72 + 72 + 4*gap
    x1 = cx - total1//2

    btn_astar = Btn(x1,            8, bw1, bh, "1: A*")
    btn_gbfs  = Btn(x1+bw1+gap,    8, bw1, bh, "2: Greedy")
    btn_ucs   = Btn(x1+2*(bw1+gap),8, bw1, bh, "3: UCS")
    btn_run   = Btn(x1+3*(bw1+gap)+gap, 8, 72, bh, "R: Run")
    btn_clear = Btn(x1+3*(bw1+gap)+gap+78, 8, 72, bh, "C: Clear")

    # Row 2 — heuristic + newmap + dynamic
    bw2 = 118
    total2 = 2*bw2 + 90 + 110 + 3*gap
    x2 = cx - total2//2

    btn_manh  = Btn(x2,              40, bw2, 22, "M: Manhattan")
    btn_eucl  = Btn(x2+bw2+gap,      40, bw2, 22, "E: Euclidean")
    btn_newmap= Btn(x2+2*(bw2+gap),  40, 90,  22, "N: NewMap")
    btn_dyn   = Btn(x2+2*(bw2+gap)+96, 40, 110, 22, "D: Dynamic OFF")

    all_btns = [btn_astar, btn_gbfs, btn_ucs, btn_run, btn_clear,
                btn_manh, btn_eucl, btn_newmap, btn_dyn]

    running = True
    while running:
        clock.tick(60)
        now = time.time()

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                p = event.pos
                if   btn_astar.hit(p):  algo = "astar"
                elif btn_gbfs.hit(p):   algo = "gbfs"
                elif btn_ucs.hit(p):    algo = "ucs"
                elif btn_run.hit(p):    start_search()
                elif btn_clear.hit(p):  clear_search()
                elif btn_newmap.hit(p): new_map(); clear_search()
                elif btn_manh.hit(p):   hfunc = manhattan; hname = "Manhattan"
                elif btn_eucl.hit(p):   hfunc = euclidean; hname = "Euclidean"
                elif btn_dyn.hit(p):
                    dyn_mode = not dyn_mode
                    btn_dyn.label = f"D: Dynamic {'ON ' if dyn_mode else 'OFF'}"
                elif in_grid(*p):
                    c, r = px_cell(*p); node = (c, r)
                    if node != START and node != GOAL:
                        if event.button == 1: walls.add(node)
                        elif event.button == 3: walls.discard(node)

            if event.type == pygame.KEYDOWN:
                k = event.key
                if k==pygame.K_r: start_search()
                if k==pygame.K_c: clear_search()
                if k==pygame.K_n: new_map(); clear_search()
                if k==pygame.K_1: algo="astar"
                if k==pygame.K_2: algo="gbfs"
                if k==pygame.K_3: algo="ucs"
                if k==pygame.K_m: hfunc=manhattan; hname="Manhattan"
                if k==pygame.K_e: hfunc=euclidean; hname="Euclidean"
                if k==pygame.K_d:
                    dyn_mode=not dyn_mode
                    btn_dyn.label=f"D: Dynamic {'ON ' if dyn_mode else 'OFF'}"

        # Drag walls
        mb = pygame.mouse.get_pressed()
        if mb[0] or mb[2]:
            mx, my = pygame.mouse.get_pos()
            if in_grid(mx, my):
                c, r = px_cell(mx, my); node = (c, r)
                if node != START and node != GOAL:
                    if mb[0]: walls.add(node)
                    if mb[2]: walls.discard(node)

        # Auto-step search
        if searching and now - last_step >= DELAY:
            step_search(); last_step = now

        # Dynamic mode — walls change periodically
        if dyn_mode:
            dyn_timer += 1
            if dyn_timer >= DYN_INTERVAL:
                dyn_timer = 0
                # Remove 2-3 random walls, add 2-3 new ones
                for _ in range(random.randint(2,3)):
                    if walls: walls.discard(random.choice(list(walls)))
                for _ in range(random.randint(2,3)):
                    c=random.randint(0,COLS-1); r=random.randint(0,ROWS-1)
                    node=(c,r)
                    if node!=START and node!=GOAL: walls.add(node)
                # Replan if path is now blocked
                if path and any(n in walls for n in path):
                    replans+=1; start_search()

        # ══ DRAW ══════════════════════════════════════
        screen.fill(DARKBG)

        # Toolbar background
        pygame.draw.rect(screen, TOOLBAR, (0, 0, WIDTH, GRID_Y - 5))

        # Grid cells
        for r in range(ROWS):
            for c in range(COLS):
                node=(c,r); x,y=cell_px(c,r)
                if   node==START:        col=START_C
                elif node==GOAL:         col=GOAL_C
                elif node in walls:      col=WALL
                elif node in path:       col=PATH_C
                elif node in visited:    col=VISITED_C
                elif node in frontier:   col=FRONTIER_C
                else:                    col=EMPTY
                pygame.draw.rect(screen, col, (x+1, y+1, CELL-2, CELL-2), border_radius=2)

        # Grid lines
        for c in range(COLS+1):
            x=GRID_X+c*CELL
            pygame.draw.line(screen,GRIDLINE,(x,GRID_Y),(x,GRID_Y+ROWS*CELL))
        for r in range(ROWS+1):
            y=GRID_Y+r*CELL
            pygame.draw.line(screen,GRIDLINE,(GRID_X,y),(GRID_X+COLS*CELL,y))

        # S / G labels
        sx,sy=cell_px(*START); gx,gy=cell_px(*GOAL)
        screen.blit(fmd.render("S",True,BLACK),(sx+5,sy+3))
        screen.blit(fmd.render("G",True,WHITE),(gx+5,gy+3))

        # Buttons
        btn_astar.draw(screen, fsm, algo=="astar")
        btn_gbfs.draw(screen,  fsm, algo=="gbfs")
        btn_ucs.draw(screen,   fsm, algo=="ucs")
        btn_run.draw(screen,   fsm, searching)
        btn_clear.draw(screen, fsm, False)
        btn_manh.draw(screen,  fsm, hfunc==manhattan)
        btn_eucl.draw(screen,  fsm, hfunc==euclidean)
        btn_newmap.draw(screen,fsm, False)
        btn_dyn.draw(screen,   fsm, dyn_mode)

        # Status bar (below grid)
        bar_y = GRID_Y + ROWS*CELL + 8
        if searching:
            stxt=f"Searching...  {algo.upper()} + {hname}"; scol=YELLOW
        elif path:
            stxt=f"Path Found!  Cost={path_cost}  Nodes={nodes_exp}  Time={elapsed:.3f}s  Replans={replans}"
            scol=GREEN
        elif done:
            stxt="No path found!"; scol=RED
        else:
            stxt=f"Ready — Press R to run   |   {algo.upper()} + {hname}"; scol=DIM

        # Status background
        pygame.draw.rect(screen, TOOLBAR,
                         (0, bar_y-4, WIDTH, 30))
        screen.blit(fmd.render(stxt, True, scol),
                    fmd.render(stxt, True, scol).get_rect(center=(WIDTH//2, bar_y+8)))

        # Mini legend (inline, below status)
        legend_y = bar_y + 26
        items = [(START_C,"Start"),(GOAL_C,"Goal"),(PATH_C,"Path"),
                 (VISITED_C,"Visited"),(FRONTIER_C,"Frontier"),(WALL,"Wall")]
        total_w = len(items)*110
        lx = (WIDTH - total_w)//2
        for col, lbl in items:
            pygame.draw.rect(screen, col, (lx, legend_y, 12, 12), border_radius=2)
            screen.blit(fsm.render(lbl, True, DIM), (lx+16, legend_y))
            lx += 110

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()