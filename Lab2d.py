# ============================================================
# ICS 556 - Autonomous Robotics
# Lab 2 - Part D: S-Trajectory Position Estimation
#
# Differential-drive forward kinematics + numerical integration
#
# Robot configuration from Part C / Lab 1:
#   Left motor  -> Port 1
#   Right motor -> Port 10 (reversed)
#
# Wheel diameter = 10.16 cm
# Wheel radius   = 5.08 cm
# Wheelbase      = 30.2 cm
#
# S trajectory:
#   Upper 3/4 circle: counter-clockwise
#   Lower 3/4 circle: clockwise
#   Radius = 50 cm
#   Total time = 15 seconds
#
# Initial configuration:
#   x = 100 cm
#   y = 150 cm
#   theta = pi/2 rad
#
# Numerical integration:
#   Euler method
#   Nominal timestep = 0.01 s
# ============================================================

from vex import *
import math


# ============================================================
# BRAIN
# ============================================================

brain = Brain()


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

# These are the wheel speeds calculated in Part C.
#
# First arc:
#   Left  = 41.2205 RPM
#   Right = 76.8898 RPM
#
# Second arc:
#   Left  = 76.8898 RPM
#   Right = 41.2205 RPM

LEFT_FIRST_RPM = 41.220472
RIGHT_FIRST_RPM = 76.889764

LEFT_SECOND_RPM = 76.889764
RIGHT_SECOND_RPM = 41.220472


# ============================================================
# HELPER FUNCTION:
# CONVERT RPM TO RAD/S
# ============================================================

def rpm_to_rad_s(rpm):
    return rpm * (2.0 * math.pi / 60.0)


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def forward_kinematics(left_rad_s, right_rad_s):

    # Linear velocity of robot centre
    v = (
        WHEEL_RADIUS_CM
        / 2.0
        * (right_rad_s + left_rad_s)
    )

    # Angular velocity of robot
    omega = (
        WHEEL_RADIUS_CM
        / WHEELBASE_CM
        * (right_rad_s - left_rad_s)
    )

    return v, omega


# ============================================================
# NUMERICAL INTEGRATION
# ============================================================

def integrate_pose(v, omega, dt):

    global x
    global y
    global theta

    # Euler integration
    x = x + v * math.cos(theta) * dt
    y = y + v * math.sin(theta) * dt

    theta = theta + omega * dt


# ============================================================
# RUN ONE ARC
# ============================================================

def run_arc(duration_s, commanded_left_rpm, commanded_right_rpm):

    global x
    global y
    global theta

    # Start both motors at the calculated Part C speeds.
    left_motor.spin(
        FORWARD,
        commanded_left_rpm,
        RPM
    )

    right_motor.spin(
        FORWARD,
        commanded_right_rpm,
        RPM
    )

    # Use the Brain timer to measure the actual elapsed time.
    brain.timer.clear()

    previous_time = brain.timer.time(SECONDS)
    elapsed = 0.0

    while elapsed < duration_s:

        # Nominal integration interval.
        wait(DT_MS, MSEC)

        current_time = brain.timer.time(SECONDS)

        dt = current_time - previous_time

        previous_time = current_time

        # Prevent the final integration interval from
        # exceeding the requested 7.5 seconds.
        remaining = duration_s - elapsed

        if dt > remaining:
            dt = remaining

        if dt <= 0:
            continue

        # ----------------------------------------------------
        # READ ACTUAL MOTOR VELOCITIES
        # ----------------------------------------------------

        left_rpm_actual = left_motor.velocity(RPM)
        right_rpm_actual = right_motor.velocity(RPM)

        # ----------------------------------------------------
        # CONVERT TO RAD/S
        # ----------------------------------------------------

        left_rad_s = rpm_to_rad_s(left_rpm_actual)
        right_rad_s = rpm_to_rad_s(right_rpm_actual)

        # ----------------------------------------------------
        # FORWARD KINEMATICS
        # ----------------------------------------------------

        v, omega = forward_kinematics(
            left_rad_s,
            right_rad_s
        )

        # ----------------------------------------------------
        # INTEGRATE POSITION AND ORIENTATION
        # ----------------------------------------------------

        integrate_pose(
            v,
            omega,
            dt
        )

        elapsed = elapsed + dt


    # Stop this arc before changing the wheel velocities.
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)


# ============================================================
# DISPLAY INITIAL VALUES
# ============================================================

brain.screen.clear_screen()

brain.screen.print("ICS 556 - Part D")
brain.screen.new_line()

brain.screen.print("Starting...")
brain.screen.new_line()

brain.screen.print("x = ")
brain.screen.print(x)
brain.screen.new_line()

brain.screen.print("y = ")
brain.screen.print(y)
brain.screen.new_line()

brain.screen.print("theta = ")
brain.screen.print(theta)

wait(2, SECONDS)


# ============================================================
# FIRST ARC
#
# 3/4 circle
# Counter-clockwise
# 7.5 seconds
# ============================================================

run_arc(
    ARC_TIME_S,
    LEFT_FIRST_RPM,
    RIGHT_FIRST_RPM
)


# ============================================================
# SECOND ARC
#
# 3/4 circle
# Clockwise
# 7.5 seconds
# ============================================================

run_arc(
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
# NORMALIZE ORIENTATION
#
# Convert theta to an equivalent angle in [-pi, pi].
# ============================================================

theta_normalized = math.atan2(
    math.sin(theta),
    math.cos(theta)
)


# ============================================================
# CALCULATE ERROR FROM THEORETICAL RESULT
# ============================================================

THEORETICAL_X = 0.0
THEORETICAL_Y = 50.0
THEORETICAL_THETA = math.pi / 2.0

position_error = math.sqrt(
    (x - THEORETICAL_X) ** 2
    +
    (y - THEORETICAL_Y) ** 2
)

orientation_error = (
    theta_normalized - THEORETICAL_THETA
)


# ============================================================
# DISPLAY FINAL RESULTS
# ============================================================

brain.screen.clear_screen()

brain.screen.print("FINAL ESTIMATE")
brain.screen.new_line()

brain.screen.print("X = ")
brain.screen.print(x)
brain.screen.new_line()

brain.screen.print("Y = ")
brain.screen.print(y)
brain.screen.new_line()

brain.screen.print("Theta = ")
brain.screen.print(theta_normalized)
brain.screen.new_line()

brain.screen.print("Pos error = ")
brain.screen.print(position_error)
brain.screen.new_line()

brain.screen.print("Theta error = ")
brain.screen.print(orientation_error)

wait(10, SECONDS)