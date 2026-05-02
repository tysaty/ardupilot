# Kangaroo Script — Session Notes (2026-04-30 / 2026-05-01)

Files modified:
- `scripts/kangaroo_MAV.lua` — primary active script
- `scripts/kangaroo.lua.disabled` — legacy copy kept in sync

---

## 1. Parameter Table Renumbering

**Problem:** After RANDOM/STRAIGHT/CIRCLE/RECTANGLE were added at slots 1–4, the subsequent
parameters (ALT_M, SPD_MIN, …) were still numbered starting from slot 2, causing slot conflicts.

**Fix:** Renumbered the whole table so slots are contiguous and conflict-free.

| Slot | Parameter     | Default | Range / Unit |
|------|---------------|---------|--------------|
| 1    | KANG_RANDOM   | 1       | 0/1          |
| 2    | KANG_STRAIGHT | 0       | 0/1          |
| 3    | KANG_CIRCLE   | 0       | 0/1          |
| 4    | KANG_RECTANGLE| 0       | 0/1          |
| 5    | KANG_ALT_M    | 80      | 20–300 m     |
| 6    | KANG_SPD_MIN  | 8       | 1–40 m/s     |
| 7    | KANG_SPD_MAX  | 20      | 1–40 m/s     |
| 8    | KANG_HOP_MIN  | 2       | 1–20 s       |
| 9    | KANG_HOP_MAX  | 6       | 1–20 s       |
| 10   | KANG_BOUND_M  | 500     | 50–3000 m    |
| 11   | KANG_TURN_DEG | 110     | 10–180 deg   |
| 12   | KANG_PAUSE_P  | 20      | 0–100 %      |
| 13   | KANG_PAUSE_S  | 2       | 0–10 s       |
| 14   | KANG_PRINT    | 1       | 0/1          |
| 15   | KANG_PRT_S    | 1       | 1–30 s       |
| 16   | KANG_OFS_M    | 500     | 0–20000 m    |
| 17   | KANG_SPD_REL  | 0.6     | 0–1 ratio    |
| 18   | KANG_STR_HDG  | 0       | 0–360 deg    |
| 19   | KANG_CIR_R    | 300     | 50–3000 m    |
| 20   | KANG_REC_W    | 400     | 50–3000 m    |
| 21   | KANG_REC_L    | 600     | 50–3000 m    |
| 22   | KANG_REC_HDG  | 0       | 0–360 deg    |

`param:add_table` size updated from 14 → 22.

---

## 2. Three New Motion Modes

### 2.1 Design decisions (confirmed with user)

| Question | Answer |
|----------|--------|
| Straight line at boundary | Travel indefinitely (no reversal) |
| Speed for new modes | Reuse `KANG_SPD_MAX` + existing `KANG_SPD_REL` ratio cap |
| Rectangle anchor position | Anchor (home + OFS_M offset) is at **one corner** |

### 2.2 New parameters (slots 18–22)

| Param        | Meaning |
|--------------|---------|
| `KANG_STR_HDG` | Fixed heading for straight-line mode (CW from north) |
| `KANG_CIR_R`   | Orbit radius for circle mode |
| `KANG_REC_W`   | Rectangle width (cross-track dimension) |
| `KANG_REC_L`   | Rectangle length (along-track dimension) |
| `KANG_REC_HDG` | Rectangle orientation — length axis points in this direction |

### 2.3 Architecture changes

#### `get_active_mode()` — new
Priority order: STRAIGHT > CIRCLE > RECTANGLE > RANDOM > nil (disabled).
```lua
local function get_active_mode()
    if KANG_STRAIGHT:get()  >= 1 then return "straight"  end
    if KANG_CIRCLE:get()    >= 1 then return "circle"    end
    if KANG_RECTANGLE:get() >= 1 then return "rectangle" end
    if KANG_RANDOM:get()    >= 1 then return "random"    end
    return nil
end
```

#### `get_capped_speed()` — new
Shared by all three new modes. Returns `KANG_SPD_MAX` with `KANG_SPD_REL` plane-speed cap.

#### `ensure_anchor()` — modified
All modes now compute a shared random start offset from home:
```lua
local start_offset = math_helpers.clamp(KANG_OFS_M:get(), 0, 20000)
local start_hdg    = math_helpers.random_between(0, 360)
orbit_north = math.cos(math.rad(start_hdg)) * start_offset
orbit_east  = math.sin(math.rad(start_hdg)) * start_offset
```
Then each mode initialises from that orbit point:
- **random** — target starts at orbit, random-walks from there (same as original, boundary clamps it)
- **straight** — target starts at orbit, moves in `KANG_STR_HDG` indefinitely
- **circle** — orbit is the circle centre; initial target = orbit + (CIR_R, 0) (north of centre)
- **rectangle** — orbit is anchor corner; remaining corners computed with NED rotation by `KANG_REC_HDG`

#### `integrate_target()` — now a dispatcher
Old random-walk body extracted to `integrate_random(now_ms, dt)`.
Three new functions added:

**`integrate_straight(dt)`**
```lua
speed_mps   = get_capped_speed()
heading_deg = straight_heading_deg
vn = cos(heading) * speed;  ve = sin(heading) * speed
target_north += vn*dt;  target_east += ve*dt
```
No boundary reversal — travels indefinitely.

**`integrate_circle(dt)`**
```lua
omega = speed / r
circle_angle_rad += omega * dt
target_north = orbit_north + r * cos(angle)
target_east  = orbit_east  + r * sin(angle)
target_vn = -speed * sin(angle)
target_ve  =  speed * cos(angle)
heading_deg = wrap_360(deg(angle) + 90)   -- tangent, CCW orbit
```

**`integrate_rectangle(dt)`**
Traverses four corners in order (C1→C2→C3→C4→C1).  
Corner layout at `KANG_REC_HDG = 0` (anchor at SW):
```
C1 = (orbit_n,               orbit_e)              -- anchor corner
C2 = (orbit_n + L,           orbit_e)              -- ahead (north)
C3 = (orbit_n + L - W*sin0,  orbit_e + W)          -- diagonal
C4 = (orbit_n,               orbit_e + W)           -- beside anchor
```
Corners are rotated by `KANG_REC_HDG` using standard NED heading rotation:
```
n_rot = n*cos(H) - e*sin(H)
e_rot = n*sin(H) + e*cos(H)
```
Progress is tracked as `rect_t ∈ [0,1)` along the current side; advances to next side when ≥ 1.

#### `update()` — modified
Gate changed from `KANG_ENABLE:get() < 1` to `get_active_mode() == nil`.

#### New module-level state variables
```lua
local orbit_north          = 0
local orbit_east           = 0
local straight_heading_deg = 0
local circle_angle_rad     = 0
local rect_corners         = nil
local rect_side            = 0
local rect_t               = 0.0
```

---

## 3. Bug Fixes Applied During Session

### 3.1 `KANG_ENABLE` shadowing bug (pre-existing)
In the original `kangaroo_MAV.lua`, all four mode-select variables were named `local KANG_ENABLE`.
Lua's scoping meant `KANG_ENABLE` pointed to the **last** declaration (RECTANGLE, default 0),
so `update()` always returned early and nothing ran. Fixed by giving each a unique variable name:
`KANG_RANDOM`, `KANG_STRAIGHT`, `KANG_CIRCLE`, `KANG_RECTANGLE`.

### 3.2 All new modes starting at home (0, 0)
Initial implementation put circle/rectangle/straight at `target_north=0, target_east=0` (home).
Fixed by the shared `orbit_north/east` offset described in §2.3.

### 3.3 Kangaroo appearing 20 km away
**Root cause:** `KANG_OFS_M` defaulted to 20,000 m. For random walk this was invisible because
`integrate_random` boundary-clamps the target to `KANG_BOUND_M` (500 m) on every tick.
For circle/rectangle/straight there is no such clamping, so the orbit was placed 20 km from home.

**Fix:**
- `KANG_OFS_M` default changed: 20000 → **500 m**
- Clamp floor in `ensure_anchor` changed: 500 → **0** (so `OFS_M=0` places the orbit at home)

---

## 4. Parameter Troubleshooting Notes

ArduPilot persists parameters on-disk. Stored values override code defaults.  
If unexpected mode is running after these changes, check stored values with:

```
param show KANG_*
```

To reset to defaults:
```
param set KANG_RANDOM    1
param set KANG_STRAIGHT  0
param set KANG_CIRCLE    0
param set KANG_RECTANGLE 0
param set KANG_OFS_M     500
```

The boot message now reports the active mode:
```
KANG: virtual target initialised (random)
```

---

## 5. Files Not Changed

- `modules/math_helpers.lua` — used but not modified
- `modules/param_helpers.lua` — used but not modified
- `control.lua` — reads KBUS bus, unaffected by kangaroo motion changes
- `kangaroo_plane_logger.lua` — reads KBUS bus, unaffected
