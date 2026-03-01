# Dynamic Pathfinding Agent

A real-time pathfinding visualizer built with Python and Pygame. Implements three AI search algorithms on a 20×20 interactive grid with dynamic obstacle support.

---

## Algorithms

| Algorithm | Strategy | Optimal? | Heuristic |
|---|---|---|---|
| A* Search | f(n) = g(n) + h(n) | Yes | Manhattan / Euclidean |
| Greedy BFS | f(n) = h(n) only | No | Manhattan / Euclidean |
| UCS | f(n) = g(n) only | Yes | None |

---

## Features

- **Live visualization** — watch visited nodes, frontier, and final path animate in real time
- **Dynamic obstacles** — walls spawn/disappear automatically, agent replans on the fly
- **Interactive grid** — draw and erase walls by clicking and dragging
- **Switch algorithms** mid-run with keyboard shortcuts
- **Stats display** — nodes expanded, path cost, time elapsed, replan count

---

## Installation

```bash
# 1. Clone the repo
git clone https://github.com/YOUR_USERNAME/dynamic-pathfinding-agent.git
cd dynamic-pathfinding-agent

# 2. Install dependency
pip install pygame

# 3. Run
python pathfinding_simple.py
```

---

## Controls

| Key / Mouse | Action |
|---|---|
| `R` | Run search |
| `C` | Clear results |
| `N` | New random map |
| `D` | Toggle dynamic mode |
| `1` | Switch to A* |
| `2` | Switch to Greedy BFS |
| `3` | Switch to UCS |
| `M` | Manhattan heuristic |
| `E` | Euclidean heuristic |
| Left Click | Draw wall |
| Right Click | Erase wall |

---

## Color Legend

| Color | Meaning |
|---|---|
| 🟢 Green | Start node |
| 🔴 Red | Goal node |
| 🟡 Yellow | Final path |
| 🔵 Blue | Visited nodes |
| 🩵 Light Blue | Frontier (open list) |
| ⬛ Dark | Wall |

---


## Course

**Artificial Intelligence** — National University of Computer & Emerging Sciences, Chiniot-Faisalabad Campus
