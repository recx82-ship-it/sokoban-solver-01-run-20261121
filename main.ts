// Sokoban solver for make.fun's sokoban-solver-01-run-20261121.
//
// Design (written from scratch for this bounty; no public solver code used):
// Every level is a 1-wide hallway with 1-4 rooms off it, each room holding
// 3-4 boxes and as many goals, and a box can never leave its room (a box
// pushed through a door into the hallway can never be pushed back). So each
// room is an independent push problem and the level's push-optimal total is
// the sum of the rooms' optima. The solver therefore:
//
//   1. splits a level into its rooms (connected floor off the hallway row,
//      each room entered through its single door cell from the hallway
//      square outside it),
//   2. solves each room on its own with A* over pushes (a state is the
//      boxes' squares plus the player's reachable region; a step is one
//      push, with the player's walk to the pushing square filled in by
//      BFS), and
//   3. walks the player between rooms along the hallway and concatenates
//      the rooms' move strings.
//
// The A* heuristic is the minimum-cost bipartite matching of boxes to goals
// under push distances (each push moves one box one square, so the matching
// cost can drop by at most one per push: the heuristic is admissible and
// consistent, so the first solved state popped is push-optimal). Dead
// squares (a box that can never be pushed to any goal, found by reverse
// push reachability from the goals) and frozen 2x2 blocks of walls/boxes
// prune hopeless states. Deterministic throughout: fixed neighbour order,
// fixed tie-breaking, and every budget is a count of expanded states, never
// the wall clock (the rules forbid clock-dependent answers). If A* ever
// exhausts its state budget on a room, the solver falls back, in order, to
// weighted A*, breadth-first search over pushes, then greedy best-first
// search, all under the same deterministic budgets.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
// (Node 22; the grader runs `node /bot/main.ts` with /input and /output.)

import { mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
] as const;

// Deterministic work budgets (counts of expanded states, never the clock).
const ASTAR_CAP = 400_000;
const WEIGHTED_CAP = 400_000;
const BFS_CAP = 250_000;
const GREEDY_CAP = 400_000;
// Whole-run budget across every room of every instance.
let globalBudget = 6_000_000;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  // The outermost frame is wall whatever it shows: nothing leaves the board.
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  goals.sort((a, b) => a - b);
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// ---------------------------------------------------------------------------
// Rooms: connected floor off the hallway row (the row the player starts on).
// ---------------------------------------------------------------------------

interface Room {
  /** Walkable cells: the room's floor (door cell included) plus the hallway square outside the door. */
  walk: Uint8Array;
  cellSet: Set<number>;
  entry: number;
  door: number;
  goals: number[];
  goalSet: Set<number>;
}

function decompose(level: Level): Room[] {
  const { w, h, wall, goals, boxes, player } = level;
  const hallRow = Math.floor(player / w);
  const boxSet = new Set(boxes);
  const goalSetAll = new Set(goals);
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  for (let start = 0; start < w * h; start++) {
    if (wall[start] || seen[start]) continue;
    if (Math.floor(start / w) === hallRow) continue;
    // Flood fill one connected component of floor off the hallway row.
    const cells: number[] = [];
    const queue = [start];
    seen[start] = 1;
    for (let qi = 0; qi < queue.length; qi++) {
      const c = queue[qi]!;
      cells.push(c);
      for (const d of DIRS) {
        const n = c + d.dy * w + d.dx;
        if (n < 0 || n >= w * h || wall[n] || seen[n]) continue;
        if (Math.floor(n / w) === hallRow) continue;
        seen[n] = 1;
        queue.push(n);
      }
    }
    const cellSet = new Set(cells);
    const roomBoxes = boxes.filter((b) => cellSet.has(b));
    const roomGoals = goals.filter((g) => cellSet.has(g));
    if (roomBoxes.length === 0 && roomGoals.length === 0) continue;
    // Door cells: component cells beside a floor cell of the hallway row.
    const doors = cells.filter((c) =>
      DIRS.some((d) => {
        const n = c + d.dy * w + d.dx;
        return n >= 0 && n < w * h && Math.floor(n / w) === hallRow && !wall[n];
      }),
    );
    if (doors.length === 0) continue;
    const door = doors[0]!;
    // The hallway square outside the door: the hallway-row neighbour of it.
    let entry = -1;
    for (const d of DIRS) {
      const n = door + d.dy * w + d.dx;
      if (n >= 0 && n < w * h && Math.floor(n / w) === hallRow && !wall[n]) {
        entry = n;
        break;
      }
    }
    if (entry < 0) continue;
    const walk = new Uint8Array(w * h);
    for (const c of cells) walk[c] = 1;
    walk[entry] = 1;
    rooms.push({
      walk,
      cellSet,
      entry,
      door,
      goals: roomGoals,
      goalSet: new Set(roomGoals),
    });
  }
  // Deterministic order: by door square.
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// ---------------------------------------------------------------------------
// One room: A* over pushes.
// ---------------------------------------------------------------------------

interface RoomSearch {
  room: Room;
  w: number;
  /** pushDist[g][cell]: fewest pushes for a box on cell to reach goal g, ignoring other boxes (-1: unreachable). */
  pushDist: Int32Array[];
  /** dead[cell]: 1 when no goal is reachable from cell by pushes. */
  dead: Uint8Array;
  goalIndex: Map<number, number>;
}

function prepareRoom(room: Room, w: number, h: number): RoomSearch {
  const pushDist: Int32Array[] = [];
  const dead = new Uint8Array(w * h).fill(1);
  const goalIndex = new Map<number, number>();
  room.goals.forEach((g, j) => goalIndex.set(g, j));
  for (let j = 0; j < room.goals.length; j++) {
    const goal = room.goals[j]!;
    const dist = new Int32Array(w * h).fill(-1);
    dist[goal] = 0;
    const queue = [goal];
    for (let qi = 0; qi < queue.length; qi++) {
      const c = queue[qi]!;
      for (const d of DIRS) {
        // The box at c came from a = c - d, pushed by a player standing at c - 2d.
        const a = c - d.dy * w - d.dx;
        const s = c - 2 * d.dy * w - 2 * d.dx;
        if (a < 0 || a >= w * h || s < 0 || s >= w * h) continue;
        if (!room.walk[a] || !room.walk[s] || dist[a] >= 0) continue;
        dist[a] = dist[c]! + 1;
        queue.push(a);
      }
    }
    pushDist.push(dist);
    for (let i = 0; i < w * h; i++) if (dist[i] >= 0) dead[i] = 0;
  }
  return { room, w, pushDist, dead, goalIndex };
}

/** Minimum-cost matching of boxes to goals under pushDist; Infinity when no box can reach its goal. */
function matchingHeuristic(search: RoomSearch, boxes: number[]): number {
  const n = boxes.length;
  const goals = search.room.goals;
  // Cost matrix, then brute-force permutations (n <= 4, so at most 24).
  const cost: number[][] = [];
  for (let i = 0; i < n; i++) {
    const row: number[] = [];
    for (let j = 0; j < n; j++) row.push(search.pushDist[j]![boxes[i]!]!);
    cost.push(row);
  }
  const perm = goals.map((_, j) => j);
  let best = Infinity;
  const visit = (k: number, sum: number): void => {
    if (sum >= best) return;
    if (k === n) {
      best = sum;
      return;
    }
    for (let t = k; t < n; t++) {
      const c = cost[k]![perm[t]!]!;
      if (c < 0) continue;
      [perm[k], perm[t]] = [perm[t]!, perm[k]!];
      visit(k + 1, sum + c);
      [perm[k], perm[t]] = [perm[t]!, perm[k]!];
    }
  };
  visit(0, 0);
  return best;
}

/** True when a box off its goal sits in a frozen 2x2 block of walls and boxes. */
function frozenBlock(search: RoomSearch, boxes: number[], boxSet: Set<number>): boolean {
  const { w, room } = search;
  for (const b of boxes) {
    if (room.goalSet.has(b)) continue;
    // The four 2x2 blocks holding b: b as top-left, top-right, bottom-left, bottom-right.
    // (The column guard keeps a block from wrapping across a row edge.)
    const bx = b % w;
    const corners: number[][] = [
      ...(bx < w - 1 ? [[b, b + 1, b + w, b + w + 1]] : []),
      ...(bx > 0 ? [[b - 1, b, b + w - 1, b + w]] : []),
      ...(bx < w - 1 ? [[b - w, b - w + 1, b, b + 1]] : []),
      ...(bx > 0 ? [[b - w - 1, b - w, b - 1, b]] : []),
    ];
    for (const block of corners) {
      // A block spilling over the board's edge cannot be a frozen block.
      if (block.some((c) => c < 0 || c >= room.walk.length)) continue;
      let frozen = true;
      for (const c of block) {
        if (c === b) continue;
        // A cell counts as blocking when it is not walkable floor, or it holds a box.
        if (room.walk[c] && !boxSet.has(c)) {
          frozen = false;
          break;
        }
      }
      if (frozen) return true;
    }
  }
  return false;
}

type SearchMode = 'astar' | 'weighted' | 'bfs' | 'greedy';

interface RoomResult {
  moves: string;
  endPlayer: number;
  endBoxes: number[];
}

/**
 * Solve one room from `startPlayer` (the hallway square outside the door).
 * Returns the move string, the player's final square and the boxes' final
 * squares, or null when the search budget runs out.
 */
function solveRoom(
  search: RoomSearch,
  startBoxes: number[],
  startPlayer: number,
  mode: SearchMode,
  cap: number,
): RoomResult | null {
  const { room, w } = search;
  const n = startBoxes.length;
  const goalSet = room.goalSet;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goalSet.has(b));

  // Flood fill of the squares the player can reach from `from`, boxes aside.
  const regionOf = (from: number, boxes: number[]): { cells: Set<number>; min: number } => {
    const boxSet = new Set(boxes);
    const cells = new Set<number>([from]);
    const queue = [from];
    for (let qi = 0; qi < queue.length; qi++) {
      const c = queue[qi]!;
      for (const d of DIRS) {
        const q = c + d.dy * w + d.dx;
        if (q < 0 || q >= room.walk.length) continue;
        if (!room.walk[q] || boxSet.has(q) || cells.has(q)) continue;
        cells.add(q);
        queue.push(q);
      }
    }
    let min = Infinity;
    for (const c of cells) if (c < min) min = c;
    return { cells, min };
  };

  // BFS walk from `from` to `to` inside `region`; the move string, or null.
  const walkTo = (from: number, to: number, region: Set<number>): string | null => {
    if (from === to) return '';
    const prev = new Map<number, [number, string]>();
    const queue = [from];
    prev.set(from, [-1, '']);
    for (let qi = 0; qi < queue.length; qi++) {
      const c = queue[qi]!;
      for (const d of DIRS) {
        const q = c + d.dy * w + d.dx;
        if (!region.has(q) || prev.has(q)) continue;
        prev.set(q, [c, d.ch]);
        if (q === to) {
          let moves = '';
          let cur = q;
          while (cur !== from) {
            const [p, ch] = prev.get(cur)!;
            moves = ch + moves;
            cur = p;
          }
          return moves;
        }
        queue.push(q);
      }
    }
    return null;
  };

  // State storage: parallel arrays, states named by index.
  const sBoxes: number[][] = [];
  const sPlayer: number[] = [];
  const sG: number[] = [];
  const sParent: number[] = [];
  const sMoves: string[] = [];
  const sKey: string[] = [];
  const best = new Map<string, number>(); // key -> smallest g seen
  const closed = new Map<string, number>(); // key -> g it was expanded at
  // Binary heap of [f, -g, index]: smallest f first, deeper first on ties, then oldest.
  const heap: [number, number, number][] = [];
  const heapLess = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1]!, heap[c]!)) c++;
        if (!heapLess(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const fOf = (g: number, h: number): number =>
    mode === 'astar' ? g + h : mode === 'weighted' ? g + 2 * h : mode === 'bfs' ? g : h;

  const addState = (boxes: number[], player: number, g: number, parent: number, moves: string): void => {
    const { min } = regionOf(player, boxes);
    const key = boxes.join(',') + '#' + min;
    const seen = best.get(key);
    if (seen !== undefined && seen <= g) return;
    const closedG = closed.get(key);
    if (closedG !== undefined && closedG <= g) return;
    const h = matchingHeuristic(search, boxes);
    if (!Number.isFinite(h)) return;
    const idx = sBoxes.length;
    sBoxes.push(boxes);
    sPlayer.push(player);
    sG.push(g);
    sParent.push(parent);
    sMoves.push(moves);
    sKey.push(key);
    best.set(key, g);
    heapPush([fOf(g, h), -g, idx]);
  };

  const h0 = matchingHeuristic(search, startBoxes);
  if (!Number.isFinite(h0)) return null;
  if (startBoxes.some((b) => search.dead[b])) return null;
  addState(startBoxes.slice().sort((a, b) => a - b), startPlayer, 0, -1, '');

  let expanded = 0;
  while (heap.length > 0 && expanded < cap && globalBudget > 0) {
    const [, , idx] = heapPop();
    const g = sG[idx]!;
    const key = sKey[idx]!;
    if (best.get(key) !== g) continue; // superseded by a cheaper route to the same state
    const closedG = closed.get(key);
    if (closedG !== undefined && closedG <= g) continue;
    closed.set(key, g);
    const boxes = sBoxes[idx]!;
    const player = sPlayer[idx]!;
    if (solved(boxes)) {
      let moves = '';
      let cur = idx;
      while (cur >= 0) {
        moves = sMoves[cur]! + moves;
        cur = sParent[cur]!;
      }
      return { moves, endPlayer: player, endBoxes: boxes };
    }
    expanded++;
    globalBudget--;
    const boxSet = new Set(boxes);
    const region = regionOf(player, boxes).cells;
    // One push per box per direction: the player must stand on the square behind the box.
    for (const b of boxes) {
      for (const d of DIRS) {
        const pushFrom = b - d.dy * w - d.dx;
        const target = b + d.dy * w + d.dx;
        if (!region.has(pushFrom)) continue;
        if (target < 0 || target >= room.walk.length) continue;
        if (!room.walk[target] || boxSet.has(target)) continue;
        const walk = walkTo(player, pushFrom, region);
        if (walk === null) continue;
        const nb = boxes.slice();
        nb[nb.indexOf(b)] = target;
        nb.sort((a, b2) => a - b2);
        if (nb.some((x) => search.dead[x])) continue;
        if (frozenBlock(search, nb, new Set(nb))) continue;
        addState(nb, b, g + 1, idx, walk + d.ch);
      }
    }
  }
  return null;
}

function solveRoomWithFallbacks(
  search: RoomSearch,
  startBoxes: number[],
  startPlayer: number,
): RoomResult | null {
  return (
    solveRoom(search, startBoxes, startPlayer, 'astar', ASTAR_CAP) ??
    solveRoom(search, startBoxes, startPlayer, 'weighted', WEIGHTED_CAP) ??
    solveRoom(search, startBoxes, startPlayer, 'bfs', BFS_CAP) ??
    solveRoom(search, startBoxes, startPlayer, 'greedy', GREEDY_CAP)
  );
}

// ---------------------------------------------------------------------------
// Whole level: walk between rooms, solve each, concatenate.
// ---------------------------------------------------------------------------

/** Walk from `from` to `to` over hallway squares and the room the player is in, boxes aside. */
function transitWalk(
  level: Level,
  hallRow: number,
  from: number,
  to: number,
  boxes: number[],
  currentRoom: Set<number> | null,
): string | null {
  if (from === to) return '';
  const { w, h, wall } = level;
  const boxSet = new Set(boxes);
  const allowed = (c: number): boolean => {
    if (c < 0 || c >= w * h || wall[c] || boxSet.has(c)) return false;
    if (Math.floor(c / w) === hallRow) return true;
    if (currentRoom !== null && currentRoom.has(c)) return true;
    return false;
  };
  if (!allowed(from) || !allowed(to)) return null;
  const prev = new Map<number, [number, string]>();
  const queue = [from];
  prev.set(from, [-1, '']);
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi]!;
    for (const d of DIRS) {
      const q = c + d.dy * w + d.dx;
      if (!allowed(q) || prev.has(q)) continue;
      prev.set(q, [c, d.ch]);
      if (q === to) {
        let moves = '';
        let cur = q;
        while (cur !== from) {
          const [p, ch] = prev.get(cur)!;
          moves = ch + moves;
          cur = p;
        }
        return moves;
      }
      queue.push(q);
    }
  }
  return null;
}

function solveLevel(text: string): string {
  const level = parseLevel(text);
  const hallRow = Math.floor(level.player / level.w);
  const rooms = decompose(level);
  if (rooms.length === 0) return '';
  let boxes = level.boxes.slice();
  let player = level.player;
  let moves = '';
  let currentRoom: Set<number> | null = null;
  const pending = rooms.slice();
  while (pending.length > 0) {
    // Take the first room the player can still walk to (deterministic order).
    let picked = -1;
    let transit = '';
    for (let i = 0; i < pending.length; i++) {
      const t = transitWalk(level, hallRow, player, pending[i]!.entry, boxes, currentRoom);
      if (t !== null) {
        picked = i;
        transit = t;
        break;
      }
    }
    if (picked < 0) return ''; // no room reachable: give up on this level
    const room = pending.splice(picked, 1)[0]!;
    moves += transit;
    player = room.entry;
    const roomBoxes = boxes.filter((b) => room.cellSet.has(b)).sort((a, b) => a - b);
    const search = prepareRoom(room, level.w, level.h);
    const result = solveRoomWithFallbacks(search, roomBoxes, player);
    if (result === null) return ''; // room unsolved within budget: give up on this level
    moves += result.moves;
    player = result.endPlayer;
    boxes = boxes.filter((b) => !room.cellSet.has(b)).concat(result.endBoxes).sort((a, b) => a - b);
    currentRoom = room.cellSet;
  }
  return moves;
}

// ---------------------------------------------------------------------------
// Entry point: one solution.txt per instance directory.
// ---------------------------------------------------------------------------

for (const id of readdirSync(INPUT_DIR).sort()) {
  const dir = join(INPUT_DIR, id);
  try {
    if (!statSync(dir).isDirectory()) continue;
    const levelPath = join(dir, 'level.txt');
    const text = readFileSync(levelPath, 'utf8');
    let moves = '';
    try {
      moves = solveLevel(text);
    } catch {
      moves = '';
    }
    mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
    writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
  } catch {
    // Still leave a well-formed (empty) answer behind rather than none.
    try {
      mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
      writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), '\n');
    } catch {
      // Nothing more can be done for this instance.
    }
  }
}
