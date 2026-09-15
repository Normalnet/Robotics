# ---------------------------------------------------------------------------- #
#                                                                              #
#       Module:       main.py                                                  #
#       Author:       Abubakari Sadik Osman                                     #
#       Created:      9/10/2026, 7:10:05 AM                                    #
#       Description:  Movement Library - Lab 1                                 #
#                                                                              #
# ---------------------------------------------------------------------------- #

# LIBRARY IMPORTS
from vex import *
from math import pi as PI

# ROBOT CONFIGURATION
brain = Brain()


right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)

# ROBOT MEASUREMENTS 
BASELINE_CM = 28.8 
WHEEL_RADIUS_CM = 5.0

WHEEL_CIRCUMFERENCE_CM = 2 * PI * WHEEL_RADIUS_CM


#  DRIVE STRAIGHT
def drive_straight(distance_cm, speed_percent):

    if speed_percent == 0:
        return

    # Calculate wheel rotation in degrees.
    wheel_degrees = (
        distance_cm / WHEEL_CIRCUMFERENCE_CM
    ) * 360

    # Determine direction.
    if speed_percent > 0:
        direction = FORWARD
        speed = speed_percent
    else:
        direction = REVERSE
        speed = abs(speed_percent)

    # Start both sides at the same time.
    left_motor.spin_for(
        direction,
        wheel_degrees,
        DEGREES,
        speed,
        PERCENT,
        wait=False
    )

    right_motor.spin_for(
        direction,
        wheel_degrees,
        DEGREES,
        speed,
        PERCENT
    )


# TURN LEFT
def turn_left(angle, speed_percent):

    if angle <= 0 or speed_percent <= 0:
        return

    # Arc distance of the moving wheel.
    wheel_degrees = (
        2 * PI * BASELINE_CM * angle
        / WHEEL_CIRCUMFERENCE_CM
    )

    # Left wheel remains stationary.
    left_motor.stop()

    # Right wheel moves forward.
    right_motor.spin_for(
        FORWARD,
        wheel_degrees,
        DEGREES,
        speed_percent,
        PERCENT
    )


# TURN RIGHT
def turn_right(angle, speed_percent):

    if angle <= 0 or speed_percent <= 0:
        return

    wheel_degrees = (
        2 * PI * BASELINE_CM * angle
        / WHEEL_CIRCUMFERENCE_CM
    )

    # Right wheel remains stationary.
    right_motor.stop()

    # Left wheel moves forward.
    left_motor.spin_for(
        FORWARD,
        wheel_degrees,
        DEGREES,
        speed_percent,
        PERCENT
    )


# SPIN LEFT
def spin_left(angle, speed_percent):

    if angle <= 0 or speed_percent <= 0:
        return

    wheel_degrees = (
        PI * BASELINE_CM * angle
        / WHEEL_CIRCUMFERENCE_CM
    )

    # Left wheel backward.
    left_motor.spin_for(
        REVERSE,
        wheel_degrees,
        DEGREES,
        speed_percent,
        PERCENT,
        wait=False
    )

    # Right wheel forward.
    right_motor.spin_for(
        FORWARD,
        wheel_degrees,
        DEGREES,
        speed_percent,
        PERCENT
    )


# SPIN RIGHT
def spin_right(angle, speed_percent):

    if angle <= 0 or speed_percent <= 0:
        return

    wheel_degrees = (
        PI * BASELINE_CM * angle
        / WHEEL_CIRCUMFERENCE_CM
    )

    # Left wheel forward.
    left_motor.spin_for(
        FORWARD,
        wheel_degrees,
        DEGREES,
        speed_percent,
        PERCENT,
        wait=False
    )

    # Right wheel backward.
    right_motor.spin_for(
        REVERSE,
        wheel_degrees,
        DEGREES,
        speed_percent,
        PERCENT
    )


# TEST THE MOVEMENT LIBRARY
def main():
    # Square pattern.
    for i in range(4):
        drive_straight(50, 40)
        turn_left(90, 20)

    # # Hexagon pattern.
    # for j in range(6):
    #     drive_straight(50, 40)
    #     spin_left(60, 30)


main()