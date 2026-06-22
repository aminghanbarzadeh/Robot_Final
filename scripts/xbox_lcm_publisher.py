#!/usr/bin/env python3
import time
import lcm
import pygame
import sys
import os

# Add lcm-types/python to sys.path so we can import gamepad_lcmt
sys.path.append(os.path.join(os.path.dirname(__file__), '../lcm-types/python'))

try:
    from gamepad_lcmt import gamepad_lcmt
except ImportError:
    print("Could not import gamepad_lcmt. Please make sure LCM types are compiled for Python.")
    sys.exit(1)

def main():
    # Initialize Pygame and the joystick module
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick found. Please connect an Xbox controller.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to joystick: {joystick.get_name()}")

    # Initialize LCM
    lc = lcm.LCM()

    # Run loop at ~50Hz
    clock = pygame.time.Clock()

    while True:
        pygame.event.pump()

        msg = gamepad_lcmt()

        # In pygame, button indices can vary by OS/driver.
        # Typical mapping for Xbox on Linux/Windows:
        # A=0, B=1, X=2, Y=3
        # LeftBumper=4, RightBumper=5
        # Back=6, Start=7
        # LeftStick=8, RightStick=9
        try:
            msg.a = joystick.get_button(0)
            msg.b = joystick.get_button(1)
            msg.x = joystick.get_button(2)
            msg.y = joystick.get_button(3)
            msg.leftBumper = joystick.get_button(4)
            msg.rightBumper = joystick.get_button(5)
            msg.back = joystick.get_button(6)
            msg.start = joystick.get_button(7)

            if joystick.get_numbuttons() > 8:
                msg.leftStickButton = joystick.get_button(8)
                msg.rightStickButton = joystick.get_button(9)
            else:
                msg.leftStickButton = 0
                msg.rightStickButton = 0

        except pygame.error:
            pass

        # Axes mapping typically:
        # 0: Left Stick X
        # 1: Left Stick Y
        # 2: Left Trigger (or Right Stick X depending on OS)
        # 3: Right Stick X (or Right Stick Y)
        # 4: Right Stick Y (or Right Trigger)
        # 5: Right Trigger

        # Mapping to typical gamepad_lcmt values (expecting -1 to 1)
        # We will negate the Y axes because typically up is negative in Pygame
        # but gamepad_lcmt might expect up to be positive or vice-versa.
        # (Users can adjust signs as needed).
        num_axes = joystick.get_numaxes()
        if num_axes >= 2:
            msg.leftStickAnalog = [joystick.get_axis(0), -joystick.get_axis(1)]
        else:
            msg.leftStickAnalog = [0.0, 0.0]

        if num_axes >= 4:
            msg.rightStickAnalog = [joystick.get_axis(2), -joystick.get_axis(3)]
        else:
            msg.rightStickAnalog = [0.0, 0.0]

        # Triggers:
        msg.leftTriggerAnalog = 0.0
        msg.rightTriggerAnalog = 0.0
        if num_axes >= 6:
            # Often left trigger is axis 4, right is axis 5 on Linux. On Windows it varies.
            msg.leftTriggerAnalog = (joystick.get_axis(4) + 1.0) / 2.0
            msg.rightTriggerAnalog = (joystick.get_axis(5) + 1.0) / 2.0

        msg.leftTriggerButton = 1 if msg.leftTriggerAnalog > 0.5 else 0
        msg.rightTriggerButton = 1 if msg.rightTriggerAnalog > 0.5 else 0

        # Publish the message
        lc.publish("interface", msg.encode())

        # Sleep to maintain loop rate
        clock.tick(50)

if __name__ == '__main__':
    main()
