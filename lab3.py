# ============================================================
# ICS 556 - Autonomous Robotics
# Lab 3
# ============================================================

from vex import *
import math

# ============================================================
# BRAIN & SENSORS
# ============================================================

brain = Brain()

# Optical Sensor (Port 20)
OPTICAL_PORT = Ports.PORT20
optical_sensor = Optical(OPTICAL_PORT)
optical_sensor.set_light(LedStateType.ON)
optical_sensor.set_light_power(50, PERCENT)

# Distance Sensor (Change PORT19 if plugged elsewhere)
DISTANCE_PORT = Ports.PORT19
distance_sensor = Distance(DISTANCE_PORT)

# Obstacle threshold in centimeters
OBSTACLE_THRESHOLD_CM = 30.0


# ============================================================
# MOTOR CONFIGURATION
# ============================================================

LEFT_PORT = Ports.PORT1
RIGHT_PORT = Ports.PORT10

left_motor = Motor(
    LEFT_PORT,
    GearSetting.RATIO_18_1,
    False
)

right_motor = Motor(
    RIGHT_PORT,
    GearSetting.RATIO_18_1,
    True
)


# ============================================================
# ROBOT PARAMETERS
# ============================================================

WHEEL_DIAMETER_CM = 10.16
WHEEL_RADIUS_CM = WHEEL_DIAMETER_CM / 2.0

WHEELBASE_CM = 30.2

TRAJECTORY_RADIUS_CM = 50.0


# ============================================================
# TRAJECTORY PARAMETERS
# ============================================================

TOTAL_TIME_S = 15.0
ARC_TIME_S = 7.5

# Numerical integration timestep
DT_S = 0.01
DT_MS = 10


# ============================================================
# INITIAL ROBOT CONFIGURATION
# ============================================================

x = 100.0
y = 150.0

# Robot initially faces upward
theta = math.pi / 2.0


# ============================================================
# PART C WHEEL VELOCITIES
# ============================================================

LEFT_FIRST_RPM = 41.220472
RIGHT_FIRST_RPM = 76.889764

LEFT_SECOND_RPM = 76.889764
RIGHT_SECOND_RPM = 41.220472


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def rpm_to_rad_s(rpm):
    return rpm * (2.0 * math.pi / 60.0)


def get_color_name(hue_val):
    """
    Classifies standard colors based on hue angle (0 - 360 degrees).
    """
    if hue_val < 30 or hue_val >= 330:
        return "RED"
    elif 30 <= hue_val < 90:
        return "YELLOW"
    elif 90 <= hue_val < 165:
        return "GREEN"
    elif 165 <= hue_val < 260:
        return "BLUE"
    else:
        return "OTHER"


def forward_kinematics(left_rad_s, right_rad_s):
    v = (WHEEL_RADIUS_CM / 2.0) * (right_rad_s + left_rad_s)
    omega = (WHEEL_RADIUS_CM / WHEELBASE_CM) * (right_rad_s - left_rad_s)
    return v, omega


def integrate_pose(v, omega, dt):
    global x, y, theta

    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += omega * dt


# ============================================================
# RUN ONE ARC WITH PAUSE & DETECTION
# ============================================================

def run_arc(arc_name, duration_s, commanded_left_rpm, commanded_right_rpm):
    global x, y, theta

    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Phase: %s" % arc_name)

    # Start motors initially
    left_motor.spin(FORWARD, commanded_left_rpm, RPM)
    right_motor.spin(FORWARD, commanded_right_rpm, RPM)

    brain.timer.clear()
    previous_time = brain.timer.time(SECONDS)
    elapsed = 0.0
    loop_tick = 0

    while elapsed < duration_s:
        wait(DT_MS, MSEC)

        # ----------------------------------------------------
        # 1. OBSTACLE PAUSE CHECK
        # ----------------------------------------------------
        # Read distance in cm (Returns large value or max if nothing in front)
        dist_cm = distance_sensor.object_distance(DistanceUnits.CM)
        
        # If obstacle is detected within 30 cm, halt and wait
        if distance_sensor.is_object_detected() and dist_cm <= OBSTACLE_THRESHOLD_CM:
            # Stop the robot
            left_motor.stop(BRAKE)
            right_motor.stop(BRAKE)

            # Freeze until the obstacle is removed
            while distance_sensor.is_object_detected() and distance_sensor.object_distance(DistanceUnits.CM) <= OBSTACLE_THRESHOLD_CM:
                # Keep showing sensor/color readout while paused
                hue = optical_sensor.hue()
                color_detected = get_color_name(hue) if optical_sensor.is_near_object() else "NONE"
                
                brain.screen.set_cursor(2, 1)
                brain.screen.print("** PAUSED - OBSTACLE **")
                brain.screen.set_cursor(3, 1)
                brain.screen.print("Dist: %-5.1f cm          " % distance_sensor.object_distance(DistanceUnits.CM))
                brain.screen.set_cursor(4, 1)
                brain.screen.print("Color: %-6s (Hue: %-3.0f)" % (color_detected, hue))
                brain.screen.set_cursor(5, 1)
                brain.screen.print("Pose: X=%.1f Y=%.1f     " % (x, y))
                
                wait(50, MSEC)

            # Resume motor command after obstacle clears
            left_motor.spin(FORWARD, commanded_left_rpm, RPM)
            right_motor.spin(FORWARD, commanded_right_rpm, RPM)

            # Reset previous_time to eliminate phantom dt accumulation during pause
            previous_time = brain.timer.time(SECONDS)
            continue

        # ----------------------------------------------------
        # 2. TIMING CALCULATION
        # ----------------------------------------------------
        current_time = brain.timer.time(SECONDS)
        dt = current_time - previous_time
        previous_time = current_time

        remaining = duration_s - elapsed
        if dt > remaining:
            dt = remaining

        if dt <= 0:
            continue

        # ----------------------------------------------------
        # 3. DEAD RECKONING ODOMETRY
        # ----------------------------------------------------
        left_rpm_actual = left_motor.velocity(RPM)
        right_rpm_actual = right_motor.velocity(RPM)

        left_rad_s = rpm_to_rad_s(left_rpm_actual)
        right_rad_s = rpm_to_rad_s(right_rpm_actual)

        v, omega = forward_kinematics(left_rad_s, right_rad_s)
        integrate_pose(v, omega, dt)
        elapsed += dt

        # ----------------------------------------------------
        # 4. SENSOR SAMPLING & SCREEN UPDATE
        # ----------------------------------------------------
        loop_tick += 1
        if loop_tick % 10 == 0:
            is_near = optical_sensor.is_near_object()
            hue = optical_sensor.hue()
            color_detected = get_color_name(hue) if is_near else "NONE"

            brain.screen.set_cursor(2, 1)
            brain.screen.print("Time: %.1f / %.1fs    " % (elapsed, duration_s))
            brain.screen.set_cursor(3, 1)
            brain.screen.print("Dist: %-5.1f cm          " % dist_cm)
            brain.screen.set_cursor(4, 1)
            brain.screen.print("Near: %-3s Color: %-6s" % ("YES" if is_near else "NO", color_detected))
            brain.screen.set_cursor(5, 1)
            brain.screen.print("X: %-5.1f Y: %-5.1f     " % (x, y))

    # Stop motors firmly
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)


# ============================================================
# DISPLAY INITIAL VALUES
# ============================================================

brain.screen.clear_screen()
brain.screen.set_font(FontType.MONO20)

brain.screen.set_cursor(1, 1)
brain.screen.print("ICS 556 - Part D")
brain.screen.set_cursor(2, 1)
brain.screen.print("Starting in 2s...")
brain.screen.set_cursor(3, 1)
brain.screen.print("x0 = %.1f" % x)
brain.screen.set_cursor(4, 1)
brain.screen.print("y0 = %.1f" % y)
brain.screen.set_cursor(5, 1)
brain.screen.print("th = %.3f rad" % theta)

wait(2, SECONDS)


# ============================================================
# FIRST ARC (Counter-Clockwise, 7.5s active motion)
# ============================================================

run_arc(
    "Arc 1 (CCW)",
    ARC_TIME_S,
    LEFT_FIRST_RPM,
    RIGHT_FIRST_RPM
)


# ============================================================
# SECOND ARC (Clockwise, 7.5s active motion)
# ============================================================

run_arc(
    "Arc 2 (CW)",
    ARC_TIME_S,
    LEFT_SECOND_RPM,
    RIGHT_SECOND_RPM
)


# ============================================================
# FINAL STOP
# ============================================================

left_motor.stop(BRAKE)
right_motor.stop(BRAKE)


# ============================================================
# NORMALIZE ORIENTATION & ERROR CALCULATION
# ============================================================

theta_normalized = math.atan2(
    math.sin(theta),
    math.cos(theta)
)

THEORETICAL_X = 0.0
THEORETICAL_Y = 50.0
THEORETICAL_THETA = math.pi / 2.0

position_error = math.sqrt(
    (x - THEORETICAL_X) ** 2
    +
    (y - THEORETICAL_Y) ** 2
)

orientation_error = theta_normalized - THEORETICAL_THETA


# ============================================================
# DISPLAY FINAL RESULTS
# ============================================================

brain.screen.clear_screen()

brain.screen.set_cursor(1, 1)
brain.screen.print("FINAL ESTIMATE")

brain.screen.set_cursor(2, 1)
brain.screen.print("X: %.2f cm" % x)

brain.screen.set_cursor(3, 1)
brain.screen.print("Y: %.2f cm" % y)

brain.screen.set_cursor(4, 1)
brain.screen.print("Theta: %.3f rad" % theta_normalized)

brain.screen.set_cursor(5, 1)
brain.screen.print("Pos Err: %.2f cm" % position_error)

brain.screen.set_cursor(6, 1)
brain.screen.print("Th  Err: %.3f rad" % orientation_error)

wait(10, SECONDS)
