# ICS 556 Lab 1 - Movement Library (Encoder-Based)
from vex import *
import math

brain = Brain()

# ---------------- CONFIGURE THESE ----------------
LEFT_PORT = Ports.PORT1
RIGHT_PORT = Ports.PORT10

WHEEL_DIAMETER_CM = 10.16
BASELINE_CM = 30.0
# -------------------------------------------------

left_motor = Motor(LEFT_PORT, GearSetting.RATIO_18_1, False)
right_motor = Motor(RIGHT_PORT, GearSetting.RATIO_18_1, True)

left_motor.set_stopping(BRAKE)
right_motor.set_stopping(BRAKE)

WHEEL_CIRC_CM = math.pi * WHEEL_DIAMETER_CM


# ---------- Equations ----------

def cm_to_degrees(distance_cm):
    # d = (n / 360) * c
    # n = 360 * d / c
    return 360.0 * distance_cm / WHEEL_CIRC_CM


def pivot_degrees(angle):
    # One wheel stays still.
    # The other wheel travels an arc with radius BASELINE_CM.
    return cm_to_degrees(
        2 * math.pi * BASELINE_CM * angle / 360.0
    )


def spin_degrees(angle):
    # Both wheels move in opposite directions.
    # Each wheel travels an arc with radius BASELINE_CM / 2.
    return cm_to_degrees(
        math.pi * BASELINE_CM * angle / 360.0
    )


# ---------- Core encoder routine ----------

def _move(left_deg, right_deg, speed):

    speed = min(abs(speed), 100)

    left_motor.reset_position()
    right_motor.reset_position()

    left_done = (left_deg == 0)
    right_done = (right_deg == 0)

    # Start left motor
    if left_done:
        left_motor.stop()
    else:
        if left_deg > 0:
            left_motor.spin(FORWARD, speed, PERCENT)
        else:
            left_motor.spin(REVERSE, speed, PERCENT)

    # Start right motor
    if right_done:
        right_motor.stop()
    else:
        if right_deg > 0:
            right_motor.spin(FORWARD, speed, PERCENT)
        else:
            right_motor.spin(REVERSE, speed, PERCENT)

    # Keep checking encoder positions
    while not (left_done and right_done):

        if not left_done:
            if abs(left_motor.position(DEGREES)) >= abs(left_deg):
                left_motor.stop()
                left_done = True

        if not right_done:
            if abs(right_motor.position(DEGREES)) >= abs(right_deg):
                right_motor.stop()
                right_done = True

        wait(10, MSEC)

    # Allow robot to settle
    wait(200, MSEC)


# ---------- TASK 1: DRIVE STRAIGHT ----------

def drive_straight(distance_cm, speed):

    deg = cm_to_degrees(abs(distance_cm))

    if distance_cm < 0:
        deg = -deg

    _move(deg, deg, speed)


# ---------- TASK 2: PIVOT TURN ----------

def turn(angle, speed, direction):

    deg = pivot_degrees(abs(angle))

    if direction == "left":
        # Left wheel stays still.
        _move(0, deg, speed)

    elif direction == "right":
        # Right wheel stays still.
        _move(deg, 0, speed)


# ---------- TASK 3: SPIN IN PLACE ----------

def spin_in_place(angle, speed, direction):

    deg = spin_degrees(abs(angle))

    if direction == "left":
        # Left wheel backward.
        # Right wheel forward.
        _move(-deg, deg, speed)

    elif direction == "right":
        # Left wheel forward.
        # Right wheel backward.
        _move(deg, -deg, speed)


# ---------- TASK 4: CLOSED SHAPE ----------

def demo_triangle():

    side = 50

    # First side
    drive_straight(side, 40)

    # First corner
    spin_in_place(90, 30, "left")

    # Second side
    drive_straight(side, 40)

    # Second corner
    spin_in_place(135, 30, "left")

    # Hypotenuse
    drive_straight(side * math.sqrt(2), 40)

    # Final corner
    spin_in_place(135, 30, "left")


# =================================================
# MAIN PROGRAM
# =================================================

brain.screen.clear_screen()
brain.screen.print("ICS 556 Lab 1")
brain.screen.new_line()
brain.screen.print("Starting...")
wait(2, SECONDS)


# =================================================
# TASK 1
# =================================================

drive_straight(100, 40)


brain.screen.new_line()
brain.screen.print("Done")

# =================================================
# TASK 2
# =================================================
        
brain.screen.clear_screen()
brain.screen.print("ICS 556 Lab 1")
brain.screen.new_line()
brain.screen.print("Starting...")
wait(2, SECONDS)

turn(90, 30, "left")

brain.screen.new_line()
brain.screen.print("Done")

# =================================================
# TASK 3
# =================================================
        
brain.screen.clear_screen()
brain.screen.print("ICS 556 Lab 1")
brain.screen.new_line()
brain.screen.print("Starting...")
wait(2, SECONDS)

spin_in_place(360, 30, "left")

brain.screen.new_line()
brain.screen.print("Done")

# =================================================
# TASK 4
# =================================================
        
brain.screen.clear_screen()
brain.screen.print("ICS 556 Lab 1")
brain.screen.new_line()
brain.screen.print("Starting...")
wait(2, SECONDS)

demo_triangle()

brain.screen.new_line()
brain.screen.print("Done")







