# 🤖 Robot Navigation Simulator
### AI Search Algorithms — BFS | DFS | A*
> Built with Python · Pygame · NumPy

---

## 📦 Installation

Make sure you have **Python 3.8+** installed, then run:

```bash
pip install pygame numpy
```

---

## ▶️ Running the Simulator

```bash
python robot_navigation_simulator.py
```

---

## 🎮 Controls

| Key / Action       | Effect                              |
|--------------------|-------------------------------------|
| `SPACE`            | Start simulation / Pause / Resume   |
| `R`                | Reset animation (keep obstacles)    |
| `C`                | Clear ALL obstacles                 |
| `1`                | Switch to **BFS**                   |
| `2`                | Switch to **DFS**                   |
| `3`                | Switch to **A\***                   |
| `+` / `=`          | Increase simulation speed           |
| `-`                | Decrease simulation speed           |
| **Left Click**     | Place / Remove obstacle             |
| **Click & Drag**   | Paint multiple obstacles            |
| `S` then Click     | Set a new **Start** position        |
| `G` then Click     | Set a new **Goal** position         |
| `ESC`              | Quit                                |

---

## 🧠 Algorithms Explained

### BFS — Breadth-First Search
- Explores all neighbours at depth N before moving to depth N+1
- **Guarantees the shortest path** in unweighted grids
- Good for: finding the optimal route
- Complexity: **O(V + E)** time, **O(V)** space

### DFS — Depth-First Search
- Dives as deep as possible before backtracking
- **Does NOT guarantee shortest path**
- Good for: maze generation, exhaustive search
- Complexity: **O(V + E)** time, **O(V)** space

### A* — A-Star Search
- Uses a **heuristic** (Manhattan distance) to prioritise promising paths
- **Guarantees shortest path** and explores fewer nodes than BFS
- Best of both worlds: optimal + efficient
- Complexity: **O(E log V)** time, **O(V)** space

---

## 🎨 Colour Legend

| Colour       | Meaning           |
|--------------|-------------------|
| 🟢 Green     | Start position    |
| 🟡 Yellow    | Goal position     |
| 🔵 Blue      | Robot (animated)  |
| 🔷 Dark blue | Explored nodes    |
| 🩵 Cyan      | Final path        |
| ⬛ Dark grey  | Obstacle / Wall   |

---

## 📁 Project Structure

```
robot_navigation_simulator.py
│
├── CONSTANTS              — Colors, grid sizes, speed levels
├── create_grid()          — NumPy grid initialisation
├── set_default_map()      — Default obstacles, start, goal
├── neighbors()            — Valid 4-directional adjacency
│
├── bfs()                  — Breadth-First Search
├── dfs()                  — Depth-First Search
├── astar()                — A* with Manhattan heuristic
├── reconstruct_path()     — Trace parent map → path list
│
├── draw_grid()            — Pygame cell renderer
├── draw_panel()           — Right-side info panel
│
└── RobotSimulator         — Main state machine class
    ├── update()           — Per-frame animation stepper
    ├── start_simulation() — Run algorithm + begin animation
    └── reset_simulation() — Clear animation state
```

---

## 💡 Tips

- Paint obstacles with **click & drag** to create complex mazes
- Compare BFS vs A* on the same maze — A* explores far fewer nodes
- DFS often finds a longer, winding path — notice the difference!
- Use `+`/`-` to slow down and watch the algorithm explore step by step

---

*Made for educational purposes — demonstrating how AI search algorithms power robotic pathfinding.*
