import math
from fractions import Fraction
import matplotlib.pyplot as plt


def angle_to_pi(theta):
    ratio = theta / math.pi
    frac = Fraction(ratio).limit_denominator()

    if frac.numerator == 0:
        return "0"

    num = frac.numerator
    den = frac.denominator

    if den == 1:
        return f"{num}π"
    else:
        return f"{num}π/{den}"


def mecanum_wheel_speeds(xdot, ydot, l=1.0, theta_dot=0.0):
    theta = math.atan2(ydot, xdot)
    theta_deg_ccw = math.degrees(theta)
    theta_deg_cw = (360.0 - theta_deg_ccw) % 360.0
    theta_pi = angle_to_pi(theta)

    inv_sqrt2 = 1.0 / math.sqrt(2.0)

    v1 = (xdot * (math.cos(theta) + math.sin(theta)) +
          ydot * (math.sin(theta) - math.cos(theta))) * inv_sqrt2 - l * theta_dot

    v2 = (xdot * (math.cos(theta) - math.sin(theta)) +
          ydot * (math.sin(theta) + math.cos(theta))) * inv_sqrt2 - l * theta_dot

    v3 = (xdot * (-math.cos(theta) - math.sin(theta)) +
          ydot * (-math.sin(theta) + math.cos(theta))) * inv_sqrt2 - l * theta_dot

    v4 = (xdot * (-math.cos(theta) + math.sin(theta)) +
          ydot * (-math.sin(theta) - math.cos(theta))) * inv_sqrt2 - l * theta_dot

    return theta, theta_pi, theta_deg_ccw, theta_deg_cw, [v1, v2, v3, v4]

def omni_wheel_speeds(vx, vy, omega=0.0, L=1.0):
    s2 = math.sqrt(2) / 2.0  # = 1/√2

    # Pure omni forward kinematic equations
    vFL = s2 * (-vx + vy) + L * omega   # Front-Left
    vFR = s2 * - ( vx + vy) - L * omega   # Front-Right
    vBL = s2 * - (-vx - vy) + L * omega   # Back-Left
    vBR = s2 * ( vx - vy) - L * omega   # Back-Right

    return vFL, vFR, vBL, vBR

def plot_direction(xdot, ydot, theta_deg_cw):
    fig, ax = plt.subplots(figsize=(6, 6))

    # Draw vector arrow
    ax.arrow(0, 0, xdot, ydot,
             head_width=0.1,
             head_length=0.15,
             fc='blue',
             ec='blue')

    # Draw helper grid & origin
    ax.scatter([0], [0], color='black')
    ax.text(0, 0, "  Origin", fontsize=10)
    ax.grid(True)

    # Set axis limits
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)

    # Labels
    ax.set_xlabel("X-axis (Forward)")
    ax.set_ylabel("Y-axis (Left)")

    ax.set_title(f"Bot Movement Direction\nClockwise Angle = {theta_deg_cw:.2f}°")

    plt.axhline(0, color="gray", linewidth=0.5)
    plt.axvline(0, color="gray", linewidth=0.5)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


# Main execution
xdot = float(input("Enter x velocity: "))
ydot = float(input("Enter y velocity: "))

theta, theta_pi, ccw_deg, cw_deg, wheels = mecanum_wheel_speeds(xdot, ydot)

# Pure omni wheel speeds
vFL, vFR, vBL, vBR = omni_wheel_speeds(xdot, ydot, omega=0.0)

wheels = [vFR, vFL, vBL, vBR]  # Match your print order

print("\n=== Angle Information ===")
print(f"θ (radians)           = {theta}")
print(f"θ (π form)            = {theta_pi}")
print(f"θ (degrees CCW)       = {ccw_deg}°")
print(f"θ (degrees CLOCKWISE) = {cw_deg}°")

# print("\n=== Wheel Speed Matrix ===")
# print(f"FR = {wheels[0]}")
# print(f"FL = {wheels[1]}")
# print(f"BL = {wheels[2]}")
# print(f"BR = {wheels[3]}")

# Compute omni wheel velocities
# vFL, vFR, vBL, vBR = omni_wheel_speeds(xdot, ydot, omega=0.0)

print("\n=== Wheel Speed Matrix (Omni) ===")
print(f"FR = {vFR}")
print(f"FL = {vFL}")
print(f"BL = {vBL}")
print(f"BR = {vBR}")

# Launch visualization
plot_direction(xdot, ydot, cw_deg)
