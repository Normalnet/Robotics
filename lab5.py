# ---------------------------------------------------------------------------
from vex import *
import math

brain = Brain()

# ===========================================================================
# ROBOT CONFIGURATION
# ===========================================================================
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_36_1)
left_motor = Motor(Ports.PORT10, GearSetting.RATIO_36_1)


LEFT_SIGN = 1
RIGHT_SIGN = -1

WHEEL_DIAMETER_CM = 20.0          # calibrated (D = 180 cm). A 4" wheel is 10.16 cm:
                                  # exactly 2x suggests an 18:1 cartridge set as 36:1
WHEEL_RADIUS_CM = WHEEL_DIAMETER_CM / 2
WHEEL_CIRCUMFERENCE_CM = 2 * 3.14159265 * WHEEL_RADIUS_CM   # c = 2*pi*r

TRACK_WIDTH_CM = 30.2             # calibrated with 4 x 90 deg turns

MAX_WHEEL_RPM = 90                # 36:1 (red) cartridge tops out at 100 RPM

# ===========================================================================
# CONTROL PARAMETERS
# ===========================================================================
K_V = 1.0                 # linear gain:  v = K_V * distance_error      (1/s)
K_OMEGA = 2.0             # angular gain: omega = K_OMEGA * heading_error (1/s)
V_MAX = 20.0              # cm/s, cap on forward speed
V_MIN = 4.0               # cm/s, keeps the robot from stalling near a waypoint
OMEGA_MAX = 1.5           # rad/s, cap on turning speed
CLOSE_ENOUGH_CM = 3.0     # waypoint counts as reached inside this radius
TURN_IN_PLACE_DEG = 20    # if heading error is larger, rotate in place (v = 0)
DT_MS = 20                # control loop period (20 ms = 50 Hz)
WAYPOINT_TIMEOUT_S = 30   # give up on a waypoint after this long
                          # (longest straight leg on 6x8 is 7 tiles = 315 cm)

RUN_MOTOR_TEST = False    # True: just drive forward briefly and show encoders


CALIBRATE_DISTANCE = False
CALIBRATE_TURN = False
CAL_DISTANCE_CM = 90.0
CAL_TURN_DEG = 90
CAL_TURN_REPEAT = 4      # 4 x 90 = full circle; errors add up, so easier to see

# ===========================================================================

# ===========================================================================
CELL_SIZE = 45.0  # cm, one floor tile

GRID = [
            [0, 0, 0, 1, 0, 0],   # row 0  (far side)
            [0, 1, 0, 0, 0, 1],   # row 1
            [0, 1, 1, 0, 1, 0],   # row 2
            [0, 0, 0, 0, 1, 0],   # row 3
            [1, 1, 0, 1, 0, 0],   # row 4
            [0, 0, 0, 1, 0, 1],   # row 5
            [0, 1, 0, 0, 0, 0],   # row 6
            [0, 1, 1, 0, 1, 0],   # row 7
            [0, 0, 0, 0, 1, 0],   # row 8
            [0, 0, 1, 0, 0, 0],   # row 9  (near side, closest to you)
]



START = (9, 0)    # (row, col) where the robot is placed: near-left tile
GOAL = (0, 5)     # (row, col) where it should end up: far-right tile


RANDOM_OBSTACLES = False
RANDOM_SEED = 7
OBSTACLE_DENSITY = 0.25   # fraction of tiles that become obstacles

START_HEADING = math.pi / 2

DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]   # 4-connected


# ===========================================================================
# HELPERS
# ===========================================================================
screen_line = [0]


def show(msg):
    """Print to the console and to the V5 brain screen."""
    print(msg)
    if screen_line[0] >= 11:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        screen_line[0] = 0
    brain.screen.print(msg)
    brain.screen.new_line()
    screen_line[0] += 1


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(a):
    """Wrap an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ===========================================================================
# A* (4-connected)
# ===========================================================================
def heuristic(a, b):
    # Manhattan distance: admissible and consistent for 4-connected grids
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def in_bounds(grid, cell):
    r, c = cell
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])


def astar(grid, start, goal):
    """Return a list of (row, col) cells from start to goal, or None."""
    if not in_bounds(grid, start) or not in_bounds(grid, goal):
        return None
    if grid[start[0]][start[1]] == 1 or grid[goal[0]][goal[1]] == 1:
        return None

    open_list = [start]
    came_from = {}
    g_score = {start: 0}
    closed = set()

    while open_list:
        # Lowest f = g + h (a linear scan is fine for small grids)
        current = min(open_list, key=lambda n: g_score[n] + heuristic(n, goal))

        if current == goal:
            return reconstruct_path(came_from, current)

        open_list.remove(current)
        closed.add(current)

        for dr, dc in DIRECTIONS:
            neighbor = (current[0] + dr, current[1] + dc)
            if not in_bounds(grid, neighbor):
                continue
            if grid[neighbor[0]][neighbor[1]] == 1 or neighbor in closed:
                continue

            tentative_g = g_score[current] + 1   # each move costs one cell

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                if neighbor not in open_list:
                    open_list.append(neighbor)

    return None


# ===========================================================================
# GRID <-> WORLD CONVERSION
# ===========================================================================
def cell_to_world(cell, num_rows):
    """(row, col) -> (x, y) in cm at the CENTER of the cell."""
    r, c = cell
    x = c * CELL_SIZE + CELL_SIZE / 2.0
    y = (num_rows - 1 - r) * CELL_SIZE + CELL_SIZE / 2.0
    return (x, y)


def world_to_cell(x, y, num_rows):
    """(x, y) in cm -> (row, col) of the cell containing that point."""
    c = int(x // CELL_SIZE)
    r = num_rows - 1 - int(y // CELL_SIZE)
    return (r, c)


def keep_corners(path):
    """Drop cells that lie on a straight line; keep start, turns and goal."""
    if len(path) < 3:
        return path[:]
    out = [path[0]]
    for i in range(1, len(path) - 1):
        d_in = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        d_out = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if d_in != d_out:
            out.append(path[i])
    out.append(path[-1])
    return out


def plan_waypoints(grid, start, goal):
    """Return (start_xy, waypoints_xy, cell_path). waypoints is None if no path."""
    rows = len(grid)
    start_xy = cell_to_world(start, rows)
    path = astar(grid, start, goal)
    if path is None:
        return start_xy, None, None
    corners = keep_corners(path)
    waypoints = [cell_to_world(cell, rows) for cell in corners[1:]]
    return start_xy, waypoints, path


# ===========================================================================
# RANDOM MAP + MAP PRINTOUT
# ===========================================================================
class SimpleRandom:
    """Small seeded random generator (a linear congruential generator), so the
    code does not depend on the random module being available on the V5."""
    def __init__(self, seed):
        self.state = (seed * 2654435761 + 12345) % 2147483648

    def next_float(self):
        self.state = (1103515245 * self.state + 12345) % 2147483648
        return self.state / 2147483648.0


def make_random_grid(rows, cols, density, seed, start, goal, max_tries=200):
    """Random obstacle map with START and GOAL kept free. Tries again with the
    next random numbers until A* finds a path. Returns None if it never does."""
    rng = SimpleRandom(seed)
    for _ in range(max_tries):
        grid = []
        for r in range(rows):
            row = []
            for c in range(cols):
                row.append(1 if rng.next_float() < density else 0)
            grid.append(row)
        grid[start[0]][start[1]] = 0
        grid[goal[0]][goal[1]] = 0
        if astar(grid, start, goal) is not None:
            return grid
    return None


def print_grid(grid, start, goal, path=None):
    """Print the map to the VEXcode console, far row at the top:
    S = start, G = goal, # = obstacle, * = path, . = free."""
    on_path = set(path) if path else set()
    print("     " + " ".join(str(c) for c in range(len(grid[0]))))
    for r in range(len(grid)):
        line = ""
        for c in range(len(grid[0])):
            if (r, c) == start:
                ch = "S"
            elif (r, c) == goal:
                ch = "G"
            elif grid[r][c] == 1:
                ch = "#"
            elif (r, c) in on_path:
                ch = "*"
            else:
                ch = "."
            line += ch + " "
        print("%3d  %s" % (r, line))


# ===========================================================================
# ODOMETRY (pose estimate from wheel encoders)
# ===========================================================================
class Odometry:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        left_motor.set_position(0, DEGREES)
        right_motor.set_position(0, DEGREES)
        self.prev_left_deg = 0.0
        self.prev_right_deg = 0.0

    def update(self):
        left_deg = LEFT_SIGN * left_motor.position(DEGREES)
        right_deg = RIGHT_SIGN * right_motor.position(DEGREES)

        # Distance each wheel travelled since the last update (cm)
        d_left = (left_deg - self.prev_left_deg) / 360.0 * WHEEL_CIRCUMFERENCE_CM
        d_right = (right_deg - self.prev_right_deg) / 360.0 * WHEEL_CIRCUMFERENCE_CM
        self.prev_left_deg = left_deg
        self.prev_right_deg = right_deg

        # Differential-drive kinematics
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / TRACK_WIDTH_CM

        # Use the mid-step heading for a better estimate
        mid_theta = self.theta + d_theta / 2.0
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta = wrap_angle(self.theta + d_theta)


# ===========================================================================
# MOTOR COMMANDS
# ===========================================================================
def set_robot_velocity(v, omega):
    """
    Convert robot linear velocity v (cm/s) and angular velocity omega (rad/s)
    into wheel angular velocities and send them to the motors.
    """
    # Linear speed of each wheel (cm/s)
    v_right = v + omega * TRACK_WIDTH_CM / 2.0
    v_left = v - omega * TRACK_WIDTH_CM / 2.0

    # Wheel angular velocity: omega_wheel = v_wheel / r  (rad/s) -> RPM
    rpm_right = (v_right / WHEEL_RADIUS_CM) * 80.0 / (2 * math.pi)
    rpm_left = (v_left / WHEEL_RADIUS_CM) * 80.0 / (2 * math.pi)

    # If either wheel would exceed the motor limit, scale both down equally
    # so the robot keeps the same curvature.
    biggest = max(abs(rpm_right), abs(rpm_left))
    if biggest > MAX_WHEEL_RPM:
        scale = MAX_WHEEL_RPM / biggest
        rpm_right *= scale
        rpm_left *= scale

    right_motor.spin(FORWARD, RIGHT_SIGN * rpm_right, RPM)
    left_motor.spin(FORWARD, LEFT_SIGN * rpm_left, RPM)


def stop_robot():
    right_motor.stop(BRAKE)
    left_motor.stop(BRAKE)


# ===========================================================================
# WAYPOINT FOLLOWING (P-control, as in Lab 4)
# ===========================================================================
def go_to_waypoint(odom, wx, wy):
    """Drive to (wx, wy). Return True if reached, False on timeout."""
    turn_threshold = TURN_IN_PLACE_DEG * math.pi / 180.0
    max_steps = int(WAYPOINT_TIMEOUT_S * 1000 / DT_MS)

    for _ in range(max_steps):
        odom.update()

        # Errors between the robot and the waypoint
        dx = wx - odom.x
        dy = wy - odom.y
        distance_error = math.sqrt(dx * dx + dy * dy)

        if distance_error < CLOSE_ENOUGH_CM:
            return True

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_heading - odom.theta)

        # Proportional control
        omega = clamp(K_OMEGA * heading_error, -OMEGA_MAX, OMEGA_MAX)
        if abs(heading_error) > turn_threshold:
            v = 0.0       # face the waypoint first, so corners are not cut
        else:
            v = clamp(K_V * distance_error, V_MIN, V_MAX)

        set_robot_velocity(v, omega)
        wait(DT_MS, MSEC)

    return False


# ===========================================================================
# MAIN
# ===========================================================================
def motor_test():
    """Spin each wheel alone, then both. WATCH the robot:
    each wheel should roll so the robot moves FORWARD. If a wheel rolls
    backward, flip its LEFT_SIGN / RIGHT_SIGN."""
    show("1) LEFT wheel forward")
    left_motor.spin(FORWARD, LEFT_SIGN * 20, RPM)
    wait(1500, MSEC)
    stop_robot()
    wait(1000, MSEC)

    show("2) RIGHT wheel forward")
    right_motor.spin(FORWARD, RIGHT_SIGN * 20, RPM)
    wait(1500, MSEC)
    stop_robot()
    wait(1000, MSEC)

    show("3) BOTH: straight fwd")
    set_robot_velocity(10.0, 0.0)
    wait(2000, MSEC)
    stop_robot()
    wait(1000, MSEC)

    show("4) Turn in place: LEFT")
    set_robot_velocity(0.0, 0.8)    # +omega = counter-clockwise
    wait(2000, MSEC)
    stop_robot()
    show("Test done")


def calibrate_distance():
    """Drive straight until odometry says CAL_DISTANCE_CM, then stop."""
    odom = Odometry(0.0, 0.0, 0.0)
    show("Driving %.0f cm (odometry)" % CAL_DISTANCE_CM)
    wait(1000, MSEC)
    while odom.x < CAL_DISTANCE_CM:
        odom.update()
        set_robot_velocity(12.0, 0.0)
        wait(DT_MS, MSEC)
    stop_robot()
    show("Odometry: %.1f cm" % odom.x)
    show("Measure real distance D")
    show("new diam = %.2f * D / %.0f" % (WHEEL_DIAMETER_CM, CAL_DISTANCE_CM))


def turn_to(odom, target):
    """Turn in place to the absolute heading target (rad) using the
    same P-control on heading as the waypoint follower."""
    target = wrap_angle(target)
    tolerance = 1.0 * math.pi / 180.0
    min_omega = 0.25                      # rad/s, avoids stalling near the end
    for _ in range(int(10000 / DT_MS)):   # 10 s timeout
        odom.update()
        err = wrap_angle(target - odom.theta)
        if abs(err) < tolerance:
            break
        omega = clamp(K_OMEGA * err, -OMEGA_MAX, OMEGA_MAX)
        if abs(omega) < min_omega:
            omega = min_omega if omega > 0 else -min_omega
        set_robot_velocity(0.0, omega)
        wait(DT_MS, MSEC)
    stop_robot()
    wait(300, MSEC)
    odom.update()                         # count any coasting after the stop


def calibrate_turn():
    """Make CAL_TURN_REPEAT turns of CAL_TURN_DEG each (default 4 x 90 deg).
    The robot should end facing exactly where it started."""
    odom = Odometry(0.0, 0.0, 0.0)
    total = CAL_TURN_DEG * CAL_TURN_REPEAT
    show("%d turns of %d deg" % (CAL_TURN_REPEAT, CAL_TURN_DEG))
    wait(1000, MSEC)
    for i in range(CAL_TURN_REPEAT):
        # Absolute targets (90, 180, 270, 360...) so small stop errors don't add up
        turn_to(odom, (i + 1) * CAL_TURN_DEG * math.pi / 180.0)
        show("Turn %d: odom %.0f deg" % (i + 1, odom.theta * 180.0 / math.pi))
        wait(700, MSEC)
    show("Odometry total: %d deg" % total)
    show("Measure real total A")
    show("new track = %.2f * %d / A" % (TRACK_WIDTH_CM, total))


def main():
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)

    if RUN_MOTOR_TEST:
        motor_test()
        return
    if CALIBRATE_DISTANCE:
        calibrate_distance()
        return
    if CALIBRATE_TURN:
        calibrate_turn()
        return

    grid = GRID
    if RANDOM_OBSTACLES:
        grid = make_random_grid(len(GRID), len(GRID[0]), OBSTACLE_DENSITY,
                                RANDOM_SEED, START, GOAL)
        if grid is None:
            show("No solvable random map")
            return
    start_xy, waypoints, cell_path = plan_waypoints(grid, START, GOAL)
    print_grid(grid, START, GOAL, cell_path)  # map + path in the VEXcode console
    if waypoints is None or cell_path is None:
        show("No path found!")
        return

    print("Cells:", cell_path)              # full path in the VEXcode console
    show("Path: %d cells, %d waypoints" % (len(cell_path), len(waypoints)))
    show("Start: (%.1f, %.1f)" % start_xy)
    for wx, wy in waypoints:
        show("  -> (%.1f, %.1f)" % (wx, wy))

    odom = Odometry(start_xy[0], start_xy[1], START_HEADING)

    # (VEX's MicroPython has no enumerate(), so index the list directly)
    for i in range(len(waypoints)):
        wx, wy = waypoints[i]
        reached = go_to_waypoint(odom, wx, wy)
        if not reached:
            stop_robot()
            show("Timeout at WP %d" % (i + 1))
            return
        show("WP %d ok (%.1f, %.1f)" % (i + 1, odom.x, odom.y))

    stop_robot()
    show("Goal reached!")
    show("Pose: %.1f, %.1f, %.0f deg" % (odom.x, odom.y, odom.theta * 180.0 / math.pi))


main()
