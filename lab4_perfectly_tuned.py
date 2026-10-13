# ============================================================
# ICS 556 - Autonomous Robotics
# LAB 4 - Zig-Zag Trajectory
# Properly Tuned P-Control
# ============================================================

from vex import *
import math

brain = Brain()

# ============================================================
# MOTOR CONFIGURATION - SAME AS LAB 3
# ============================================================

left_motor = Motor(
    Ports.PORT1,
    GearSetting.RATIO_18_1,
    False
)

right_motor = Motor(
    Ports.PORT10,
    GearSetting.RATIO_18_1,
    True
)

# ============================================================
# ROBOT PARAMETERS
# ============================================================

WHEEL_DIAMETER_CM = 10.16
WHEEL_RADIUS_CM = WHEEL_DIAMETER_CM / 2.0
WHEELBASE_CM = 30.2

# ============================================================
# TUNING PARAMETERS
# ============================================================

KV = 0.8                 # LINEAR gain
K_OMEGA = 2.5            # TURNING gain

DT_MS = 100               # Control interval
CLOSE_ENOUGH_CM = 8.0    # Waypoint tolerance

MAX_WHEEL_RPM = 60.0     # Keep robot slow for small room


# ============================================================
# STARTING POSE
# Robot starts at (0,0), facing +X
# ============================================================

x = 0.0
y = 0.0
theta = 0.0


# ============================================================
# SMALL-ROOM ZIG-ZAG
# ============================================================

WAYPOINTS = [
    (80.0, 60.0),
    (160.0, 0.0),
    (240.0, 60.0),
    (320.0, 0.0)
]


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def clamp(value, minimum, maximum):

    if value > maximum:
        return maximum

    if value < minimum:
        return minimum

    return value


def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


# ============================================================
# INVERSE KINEMATICS
# ============================================================

def inverse_kinematics(v, omega):

    right_rad_s = (
        v / WHEEL_RADIUS_CM
        +
        (WHEELBASE_CM * omega)
        /
        (2.0 * WHEEL_RADIUS_CM)
    )

    left_rad_s = (
        v / WHEEL_RADIUS_CM
        -
        (WHEELBASE_CM * omega)
        /
        (2.0 * WHEEL_RADIUS_CM)
    )

    return left_rad_s, right_rad_s


def rad_s_to_rpm(rad_s):

    return rad_s * 60.0 / (2.0 * math.pi)


# ============================================================
# MOTOR CONTROL
# ============================================================

def drive_wheels(left_rpm, right_rpm):

    left_rpm = clamp(
        left_rpm,
        -MAX_WHEEL_RPM,
        MAX_WHEEL_RPM
    )

    right_rpm = clamp(
        right_rpm,
        -MAX_WHEEL_RPM,
        MAX_WHEEL_RPM
    )

    if left_rpm >= 0:
        left_motor.spin(FORWARD, left_rpm, RPM)
    else:
        left_motor.spin(REVERSE, abs(left_rpm), RPM)

    if right_rpm >= 0:
        right_motor.spin(FORWARD, right_rpm, RPM)
    else:
        right_motor.spin(REVERSE, abs(right_rpm), RPM)


def stop_robot():

    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)


# ============================================================
# FOLLOW ONE WAYPOINT
# ============================================================

def follow_waypoint(target_x, target_y):

    global x, y, theta

    while True:

        # ----------------------------------------------------
        # Position error
        # ----------------------------------------------------

        error_x = target_x - x
        error_y = target_y - y

        distance = math.sqrt(
            error_x ** 2 +
            error_y ** 2
        )

        # ----------------------------------------------------
        # Waypoint reached?
        # ----------------------------------------------------

        if distance <= CLOSE_ENOUGH_CM:

            stop_robot()
            wait(300, MSEC)
            break

        # ----------------------------------------------------
        # Desired direction
        # ----------------------------------------------------

        desired_theta = math.atan2(
            error_y,
            error_x
        )

        heading_error = normalize_angle(
            desired_theta - theta
        )

        # ----------------------------------------------------
        # P-CONTROLLER
        # ----------------------------------------------------

        v = KV * distance
        omega = K_OMEGA * heading_error

        # ----------------------------------------------------
        # Limit speed
        # ----------------------------------------------------

        max_linear_velocity = (
            WHEEL_RADIUS_CM *
            MAX_WHEEL_RPM *
            2.0 * math.pi /
            60.0
        )

        v = clamp(
            v,
            0.0,
            max_linear_velocity
        )

        # ----------------------------------------------------
        # Convert v and omega to wheel speeds
        # ----------------------------------------------------

        left_rad_s, right_rad_s = inverse_kinematics(
            v,
            omega
        )

        left_rpm = rad_s_to_rpm(left_rad_s)
        right_rpm = rad_s_to_rpm(right_rad_s)

        drive_wheels(
            left_rpm,
            right_rpm
        )

        # ----------------------------------------------------
        # Estimate robot pose
        # ----------------------------------------------------

        dt = DT_MS / 1000.0

        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += omega * dt

        theta = normalize_angle(theta)

        # ----------------------------------------------------
        # Display
        # ----------------------------------------------------

        brain.screen.clear_screen()

        brain.screen.set_cursor(1, 1)
        brain.screen.print("LAB 4 - ZIGZAG")

        brain.screen.set_cursor(2, 1)
        brain.screen.print(
            "X %.1f  Y %.1f" % (x, y)
        )

        brain.screen.set_cursor(3, 1)
        brain.screen.print(
            "Target %.0f %.0f" %
            (target_x, target_y)
        )

        brain.screen.set_cursor(4, 1)
        brain.screen.print(
            "Error %.1f cm" % distance
        )

        wait(DT_MS, MSEC)


# ============================================================
# START
# ============================================================

brain.screen.clear_screen()

brain.screen.set_cursor(1, 1)
brain.screen.print("LAB 4 ZIG-ZAG")

brain.screen.set_cursor(2, 1)
brain.screen.print("Starting...")

wait(2, SECONDS)


# ============================================================
# FOLLOW ZIG-ZAG
# ============================================================

for waypoint in WAYPOINTS:

    follow_waypoint(
        waypoint[0],
        waypoint[1]
    )


# ============================================================
# FINISH
# ============================================================

stop_robot()

brain.screen.clear_screen()

brain.screen.set_cursor(1, 1)
brain.screen.print("TRAJECTORY COMPLETE")

brain.screen.set_cursor(2, 1)
brain.screen.print(
    "X %.1f Y %.1f" % (x, y)
)

wait(5, SECONDS)
