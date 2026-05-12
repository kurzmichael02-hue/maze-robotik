# wegfindungs-algorithmen — mathematische erklärung

das maze ist ein **ungewichteter graph**:
- knoten = zellen (n × n)
- kanten = passierbare wand-übergänge zwischen nachbar-zellen
- jede kante hat kosten 1

gesucht: kürzester pfad von start `s` zu ziel `g`.

---

## 1. wall-follower (links-hand-regel)

**idee:** halte linke hand an der wand, gehe weiter. naive heuristik aus der pfadfindung der antike.

**algorithmus:**
```
loop:
  wenn links offen → links + 1 schritt
  sonst wenn geradeaus offen → 1 schritt
  sonst wenn rechts offen → rechts + 1 schritt
  sonst → umdrehen (dead-end)
```

**korrektheit:**
findet das ziel **nur in einfach zusammenhängenden mazes** (perfect maze, genau ein weg). bei mazes mit **loops oder mehreren wegen** kann er in einem inneren teil endlos kreisen und das ziel nie erreichen.

**komplexität:**
- bestes szenario: `O(n²)` schritte für n × n maze (perfect maze)
- worst case (multi-path mit inseln): nicht terminierend bzw. erreicht limit
- speicher: `O(1)`

**unsere zahlen:** bei 25 × 25 multi-path maze: 536 schritte, vergleicht mit BFS-optimum 200. heißt **2.7x länger als nötig** UND scheitert manchmal komplett (im 8x8 demo: 512 max steps ohne erreichen).

---

## 2. BFS — Breadth-First Search

**idee:** expandiere die zellen in wellen vom start aus. alle zellen in distanz `d` werden vor jeder zelle in distanz `d+1` expandiert.

**algorithmus:**
```
queue = [start]
parent[start] = null
while queue:
  u = queue.popleft()
  if u == goal: rekonstruiere pfad via parent
  für jeden nachbarn v von u:
    wenn v noch nicht besucht:
      parent[v] = u
      queue.append(v)
```

**korrektheit (theorem):**
in einem ungewichteten graphen findet BFS **garantiert den kürzesten pfad**. beweis: per induktion über die distanz `d`. alle zellen die im `d`-ten level expandiert werden haben genau distanz `d` vom start (sonst wären sie früher expandiert worden).

**komplexität:**
- zeit: `O(|V| + |E|)` — also `O(n²)` für ein maze
- speicher: `O(|V|)` für queue + parent map

**unsere zahlen:** bei 25 × 25: pfadlänge 200 (optimum), 386 expansions.

---

## 3. A* — A-Star

**idee:** wie BFS, aber priorisiere zellen die "wahrscheinlich näher am ziel" sind. dafür: prioritätswarteschlange mit `f(n) = g(n) + h(n)`.
- `g(n)` = tatsächliche kosten von start zu `n`
- `h(n)` = heuristische schätzung von `n` zu goal

**algorithmus:**
```
open = priority_queue mit (f(start), start)
g[start] = 0
while open not empty:
  u = open.pop()  # node mit kleinstem f
  if u == goal: rekonstruiere pfad
  für jeden nachbarn v:
    tentative_g = g[u] + 1
    wenn tentative_g < g[v]:
      g[v] = tentative_g
      f[v] = tentative_g + h(v)
      open.push(v)
```

**heuristik (manhattan):** `h(n) = |n.x - goal.x| + |n.y - goal.y|`

**korrektheit (theorem):**
A* findet den optimalen pfad gdw. die heuristik **admissible** ist (`h(n) ≤ tatsächliche kosten von n zum ziel`). manhattan-distanz unterschätzt die echte distanz in einem maze (da wände nur längeren weg verursachen). → admissible → A* optimal.

**warum A* schneller als BFS:**
A* expandiert keine zellen die "weg vom ziel" sind solange es bessere kandidaten gibt. die heuristik zieht die suche in richtung goal.

**komplexität:**
- worst case wie BFS (`O(|V|)`)
- praxis: deutlich weniger expansions, abhängig von heuristik-qualität

**unsere zahlen:** bei 25 × 25: pfadlänge 200 (identisch zu BFS, beweis dass beide optimal sind), aber nur 342 expansions vs BFS 386 — **11% effizienter**. bei perfect mazes (hard) ist der unterschied größer.

---

## 4. flood-fill (micromouse-stil)

**idee:** zwei phasen.
1. **distance-map** vom ziel aus per BFS — jede zelle bekommt ihre echte distanz zum goal.
2. **gradient descent** vom start zum ziel — wähle in jedem schritt den nachbarn mit kleinster distanz.

**algorithmus:**
```
# phase 1
dist[goal] = 0
queue = [goal]
while queue:
  u = queue.popleft()
  für jeden nachbarn v:
    wenn dist[v] unbestimmt:
      dist[v] = dist[u] + 1
      queue.append(v)

# phase 2
path = [start]
u = start
while u != goal:
  v = nachbar von u mit kleinstem dist[v]
  path.append(v); u = v
```

**korrektheit:**
phase 1 ist BFS rückwärts → liefert wirkliche distanzen. phase 2 ist gradientenabstieg auf einer 4-connected unweighted lattice → folgt einer geodätischen → findet den optimum.

**warum für micromouse?**
die distance-map ist **wiederverwendbar**. der bot kann mit einer karte fahren, beim entdecken neuer wände schnell die distance-map updaten ohne von vorne zu planen. ideal bei dynamic re-planning.

**komplexität:**
- phase 1: `O(|V|)` (BFS)
- phase 2: `O(pfadlänge)`
- speicher: distance-map = `O(|V|)`

**unsere zahlen:** bei 25 × 25: pfadlänge 200 (optimum), aber 625 expansions (mappt **die ganze map**, nicht nur einen pfad-bereich). nachteil: mehr arbeit pro plan. vorteil: ergebnis ist re-usable.

---

## vergleich (zusammenfassung)

| algo | optimal? | erwartete expansions | besonderheit |
|---|---|---|---|
| wall-follower | nein | O(n²) bis ∞ | scheitert in multi-path |
| BFS | ja | O(n²) | einfach, robust |
| A* | ja | O(n²), praxis weniger | schnellste pfadsuche |
| flood-fill | ja | O(n²), immer ganze map | re-usable für dynamic re-planning |

**für unser projekt:** A* gewinnt in der demo (kürzester pfad mit weniger expansions). flood-fill wäre die wahl wenn das maze sich ändert (z.b. dynamische türen).

---

## referenzen

- Cormen, Leiserson, Rivest, Stein — *Introduction to Algorithms* (CLRS), 4th ed., kapitel 22 (BFS), 24 (A*)
- Hart, Nilsson, Raphael (1968): *A Formal Basis for the Heuristic Determination of Minimum Cost Paths* — original A* paper
- micromouse community: flood-fill als de-facto standard für maze-roboter
