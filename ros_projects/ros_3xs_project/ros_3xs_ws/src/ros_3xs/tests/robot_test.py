import sys
import os
import numpy as np
from scipy.interpolate import CubicSpline
import time
# Add the parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros_3xs.robot_control import RobotController


def jog_follow_curve(
    controller, 
    control_points, 
    duration=120.0, 
    update_rate_hz=5, 
    max_speed=250, 
    min_speed=5, 
    speed_scale_factor = 0.8
):
    import numpy as np
    import matplotlib.pyplot as plt
    import time
    from scipy.interpolate import splprep, splev

    control_points = np.array(control_points)
    assert control_points.shape[1] == 2, "Control points must be 2D (x, y)"

    # --- Generate a tight spline through the control points ---
    total_steps = int(duration * update_rate_hz)
    u_new = np.linspace(0, 1, total_steps)

    x = control_points[:, 0]
    y = control_points[:, 1]
    tck, _ = splprep([x, y], s=0, k=min(3, len(control_points)-1))  # safe for small N
    x_vals, y_vals = splev(u_new, tck)
    dx_vals, dy_vals = splev(u_new, tck, der=1)

    # --- Preview the path and velocity vectors ---
    arrow_scale = 0.05 * np.hypot(*(np.ptp(control_points, axis=0)))
    norms = np.hypot(dx_vals, dy_vals)
    nonzero = norms > 1e-6
    dx_plot = np.zeros_like(dx_vals)
    dy_plot = np.zeros_like(dy_vals)
    dx_plot[nonzero] = dx_vals[nonzero] / norms[nonzero] * arrow_scale
    dy_plot[nonzero] = dy_vals[nonzero] / norms[nonzero] * arrow_scale

    plt.figure(figsize=(8, 8))
    plt.plot(x_vals, y_vals, label='Spline Path', linewidth=2)
    plt.quiver(x_vals[::20], y_vals[::20], dx_plot[::20], dy_plot[::20], 
               angles='xy', scale_units='xy', scale=1, color='blue', width=0.003, label='Velocity Vectors')
    plt.scatter(x, y, color='red', label='Control Points')
    plt.title('Preview of Path with Velocity Vectors')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

    # --- Jogging loop ---
    dt = 1.0 / update_rate_hz
    controller.start_jog()
    time.sleep(0.1)

    try:
        for i in range(total_steps):
            print("step is ", i)
            print("out of ", total_steps)
            x = x_vals[i]
            y = y_vals[i]
            print("position is: ", x, y)

            # Optional: clamp workspace to 0–4000 mm
            if not (0 <= x <= 4000 and 0 <= y <= 4000):
                print(f"⚠️ Skipping out-of-bounds point: ({x:.1f}, {y:.1f})")
                continue

            dx = dx_vals[i]
            dy = dy_vals[i]
            norm = np.hypot(dx, dy)

            if norm > 1e-6:
                vx = dx / norm * max_speed
                vy = dy / norm * max_speed
                
                # Minimum velocity threshold
                speed = np.hypot(vx, vy)
                if speed < min_speed:
                    vx *= min_speed / speed
                    vy *= min_speed / speed

                # Speed scale factor
                vx *= speed_scale_factor
                vy *= speed_scale_factor
                print("speed is: ", vx, vy)
            else:
                print("speed has gone to zero!")
                vx = vy = 0

            controller.set_jog_values(vx, vy, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(dt)

    finally:
        controller.stop_jog()



robot = RobotController(mode='simulation')

robot.connect()

#print(robot.controller.zero_all_joints())

robot.prepare_robot()

robot.controller.move_cartesian(
    500.0, 500.0, 0.0,  # x, y, z movement
    0.0, 0.0, 0.0,     # rotational movement
    0.0, 0.0, 0.0,     # tool coordinates
    300.0,              # speed
    wait_move_finished=True,
    move_finished_timeout=1000  
)

jog_follow_curve(
    robot.controller,
    control_points=[(500, 500), (2000, 3000), (3500, 3500)],
    duration=120.0,
    max_speed=200
)

# robot.controller.start_jog()
# robot.controller.set_jog_values(200, 0, 0, 0, 0, 0, 0, 0, 0)
# time.sleep(10)
# robot.controller.set_jog_values(0, 200, 0, 0, 0, 0, 0, 0, 0)
# time.sleep(10)
# robot.controller.set_jog_values(200, 0, 0, 0, 0, 0, 0, 0, 0)
# time.sleep(10)
# robot.controller.stop_jog()

print("done with motion!")

robot.cleanup()

