cd ~/workspace/fastanchor-nav-stack

cat > /tmp/fix_waypoint_launch.sh <<'BASH'
#!/usr/bin/env bash
set -eo pipefail

WS="$HOME/workspace/fastanchor-nav-stack"
cd "$WS"

set +u
source /opt/ros/humble/setup.bash
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"

LAUNCH="src/navigation_bringup/launch/navigation_system.launch.py"

echo "=================================================="
echo " Repair waypoint_mission_manager launch block"
echo "=================================================="

# ============================================================
# 1. Find latest backup
# ============================================================

BACKUP_DIR="$(
    find "$WS" \
        -maxdepth 1 \
        -type d \
        -name '.waypoint_action_backup_*' \
        -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr \
    | head -n1 \
    | cut -d' ' -f2-
)"

if [[ -z "${BACKUP_DIR:-}" ]]; then
    echo "ERROR: no .waypoint_action_backup_* directory found"
    exit 1
fi

BACKUP_LAUNCH="$BACKUP_DIR/navigation_system.launch.py"

if [[ ! -f "$BACKUP_LAUNCH" ]]; then
    echo "ERROR: backup launch file not found:"
    echo "  $BACKUP_LAUNCH"
    exit 1
fi

echo
echo "[1/6] Using backup:"
echo "  $BACKUP_DIR"

# Preserve malformed file for inspection if needed.
cp "$LAUNCH" \
   "$WS/navigation_system.launch.py.failed.$(date +%Y%m%d_%H%M%S)"

# Restore clean pre-upgrade launch.
cp "$BACKUP_LAUNCH" "$LAUNCH"

echo "Original launch file restored."

# ============================================================
# 2. Locate existing waypoint manager block
# ============================================================

echo
echo "[2/6] Patching only waypoint manager conditions..."

python3 - <<'PY'
from pathlib import Path

path = Path(
    "src/navigation_bringup/launch/navigation_system.launch.py"
)

text = path.read_text()

needle = 'executable="waypoint_mission_manager"'
pos = text.find(needle)

if pos < 0:
    raise RuntimeError(
        "waypoint_mission_manager Node was not found"
    )

# Find containing Node(...)
node_start = text.rfind("Node(", 0, pos)

if node_start < 0:
    raise RuntimeError(
        "Could not find beginning of waypoint manager Node"
    )

# Parse parentheses while respecting Python strings enough for this block.
depth = 0
quote = None
escape = False
node_end = None

for i in range(node_start, len(text)):
    ch = text[i]

    if escape:
        escape = False
        continue

    if ch == "\\" and quote is not None:
        escape = True
        continue

    if quote is not None:
        if ch == quote:
            quote = None
        continue

    if ch in ("'", '"'):
        quote = ch
        continue

    if ch == "(":
        depth += 1
    elif ch == ")":
        depth -= 1
        if depth == 0:
            node_end = i + 1
            break

if node_end is None:
    raise RuntimeError(
        "Could not find end of waypoint manager Node"
    )

block = text[node_start:node_end]

print("===== ORIGINAL MANAGER BLOCK =====")
print(block)

# ------------------------------------------------------------
# Remove:
#
# condition=IfCondition(
#     PythonExpression(["'", waypoints_file, "' != ''"])
# ),
#
# Do this structurally rather than replacing the whole Node.
# ------------------------------------------------------------

condition_key = "condition=IfCondition("

cpos = block.find(condition_key)

if cpos >= 0:
    line_start = block.rfind("\n", 0, cpos) + 1

    paren_start = block.find("(", cpos)
    depth = 0
    quote = None
    escape = False
    cond_end = None

    for i in range(paren_start, len(block)):
        ch = block[i]

        if escape:
            escape = False
            continue

        if ch == "\\" and quote is not None:
            escape = True
            continue

        if quote is not None:
            if ch == quote:
                quote = None
            continue

        if ch in ("'", '"'):
            quote = ch
            continue

        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                cond_end = i + 1

                # consume comma
                if (
                    cond_end < len(block)
                    and block[cond_end] == ","
                ):
                    cond_end += 1

                # consume the rest of that line
                while (
                    cond_end < len(block)
                    and block[cond_end] != "\n"
                ):
                    cond_end += 1

                if (
                    cond_end < len(block)
                    and block[cond_end] == "\n"
                ):
                    cond_end += 1

                break

    if cond_end is None:
        raise RuntimeError(
            "Could not parse waypoint manager condition"
        )

    block = (
        block[:line_start]
        + block[cond_end:]
    )

    print(
        "Removed waypoints_file launch condition."
    )
else:
    print(
        "No waypoint manager condition found; "
        "it may already be unconditional."
    )

# ------------------------------------------------------------
# Remove only the standalone:
#
#     waypoints_file,
#
# inside parameters=[...].
#
# Do NOT remove the LaunchConfiguration variable or global
# DeclareLaunchArgument yet. They are harmless and preserving
# them minimizes unrelated changes.
# ------------------------------------------------------------

lines = block.splitlines(keepends=True)

new_lines = []
removed_waypoint_parameter = False

for line in lines:
    if line.strip() == "waypoints_file,":
        removed_waypoint_parameter = True
        continue

    new_lines.append(line)

block = "".join(new_lines)

if removed_waypoint_parameter:
    print(
        "Removed waypoints_file from manager parameters."
    )
else:
    print(
        "No standalone waypoints_file parameter found."
    )

# Put the minimally modified Node back.
patched = (
    text[:node_start]
    + block
    + text[node_end:]
)

path.write_text(patched)

print()
print("===== PATCHED MANAGER BLOCK =====")
print(block)
PY

# ============================================================
# 3. Syntax validation
# ============================================================

echo
echo "[3/6] Python syntax validation..."

python3 -m py_compile "$LAUNCH"

echo "navigation_system.launch.py syntax: OK"

# ============================================================
# 4. Verify manager is unconditional
# ============================================================

echo
echo "[4/6] Checking final manager block..."

python3 - <<'PY'
from pathlib import Path

text = Path(
    "src/navigation_bringup/launch/navigation_system.launch.py"
).read_text()

needle = 'executable="waypoint_mission_manager"'
pos = text.find(needle)

node_start = text.rfind("Node(", 0, pos)

depth = 0
end = None

for i in range(node_start, len(text)):
    if text[i] == "(":
        depth += 1
    elif text[i] == ")":
        depth -= 1
        if depth == 0:
            end = i + 1
            break

block = text[node_start:end]

assert "condition=IfCondition" not in block, (
    "Waypoint manager is still conditional"
)

for line in block.splitlines():
    assert line.strip() != "waypoints_file,", (
        "waypoints_file is still passed to manager"
    )

print("Waypoint manager:")
print("  always launch: YES")
print("  requires waypoint YAML: NO")
PY

# ============================================================
# 5. Check manager syntax too
# ============================================================

echo
echo "[5/6] Checking dynamic Action manager..."

python3 -m py_compile \
    src/navigation_bringup/navigation_bringup/waypoint_mission_manager.py

echo "waypoint_mission_manager.py syntax: OK"

# ============================================================
# 6. Build
# ============================================================

echo
echo "[6/6] Building nav_interfaces + navigation_bringup..."

colcon build \
    --packages-select nav_interfaces navigation_bringup \
    --symlink-install

echo
echo "=================================================="
echo " SUCCESS"
echo "=================================================="
echo
echo "Launch file repaired."
echo
echo "waypoint_mission_manager now:"
echo "  - always starts"
echo "  - does not require waypoints_file"
echo "  - waits for /follow_waypoints Action goals"
echo
BASH

chmod +x /tmp/fix_waypoint_launch.sh

/tmp/fix_waypoint_launch.sh
