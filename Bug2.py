from controller import Robot
import math
import matplotlib.pyplot as plt


def get_bearing_to_goal(pos1, pos2):
    """Calculates the bearing (angle) from pos1 to pos2 in radians."""
    return math.atan2(pos2[1] - pos1[1], pos2[0] - pos1[0])

def get_robot_yaw(compass_values):
    """Calculates the robot's current yaw (heading) in radians from compass data."""
    return math.atan2(compass_values[0], compass_values[1])

def get_distance(p1, p2):
    """Calculates the Euclidean distance between two points."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def is_on_m_line(pos, m_start, goal, tol=0.1):
    """Checks if the robot is close to the M-line (line between start and goal)."""
    m_theta = get_bearing_to_goal(m_start, goal)
    r_theta = get_bearing_to_goal(m_start, pos)
    diff = abs((m_theta - r_theta + math.pi) % (2 * math.pi) - math.pi)
    return diff < tol

def follow_right_wall(dists, thresholds, speeds):
    """Implements robust right-hand wall-following logic for Bug2."""
    front_obstacle = dists[0] > thresholds['obstacle'] or dists[7] > thresholds['obstacle']
    right_wall_too_close = dists[2] > thresholds['too_close'] or dists[1] > thresholds['too_close']
    right_wall_missing = dists[2] < thresholds['obstacle'] and dists[1] < thresholds['obstacle']

    if front_obstacle:
        return -speeds['turn'], speeds['turn'] 
    elif right_wall_too_close:
        return speeds['adjust'], speeds['straight']
    elif right_wall_missing:
        return speeds['straight'], speeds['recover']
    else:
        return speeds['straight'], speeds['straight']

def follow_left_wall(dists, thresholds, speeds):
    """Implements a robust left-hand wall-following logic for the second goal."""
    front_obstacle = dists[0] > thresholds['obstacle'] or dists[7] > thresholds['obstacle']
    left_wall_too_close = dists[5] > thresholds['too_close'] or dists[6] > thresholds['too_close']
    left_wall_missing = dists[5] < thresholds['obstacle'] and dists[6] < thresholds['obstacle']

    if front_obstacle:
        return speeds['turn'], -speeds['turn']
    elif left_wall_too_close:
        return speeds['straight'], speeds['adjust']
    elif left_wall_missing:
        return speeds['adjust'], speeds['straight']
    else:
        return speeds['straight'], speeds['straight']

if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    goals = [(0, 4.7), (-3.32, 3.75)]
    goal_idx = 0
    goal = goals[goal_idx]

    state = 'GOAL_SEEKING'
    hit_point = None
    dist_at_hit = float('inf')

    trajectory, hit_points_log, leave_points_log = [], [], []

    MAX_SPEED = 6.28
    GOAL_RADIUS = 0.15
    EXIT_MARGIN = 0.1
    HIT_PROXIMITY_THR = 0.2
    WALL_THRESHOLDS = {'obstacle': 90.0, 'too_close': 120.0}
    WALL_SPEEDS = {
        'turn': 0.8 * MAX_SPEED, 'straight': MAX_SPEED,
        'adjust': 0.7 * MAX_SPEED, 'recover': 0.5 * MAX_SPEED
    }

    gps = robot.getDevice('gps'); gps.enable(timestep)
    compass = robot.getDevice('compass'); compass.enable(timestep)
    ps = [robot.getDevice(f'ps{i}') for i in range(8)]; [p.enable(timestep) for p in ps]
    lm = robot.getDevice('left wheel motor'); rm = robot.getDevice('right wheel motor')
    lm.setPosition(float('inf')); rm.setPosition(float('inf'))
    lm.setVelocity(0); rm.setVelocity(0)

    robot.step(timestep)
    gps_pos = gps.getValues()
    m_line_start = (gps_pos[0], gps_pos[1])
    print(f"Starting Mission. Algorithm: Bug2. Goal 1: {goal}")

    while robot.step(timestep) != -1:
        gps_pos = gps.getValues()
        pos = (gps_pos[0], gps_pos[1])
        if not trajectory or get_distance(pos, trajectory[-1]) > 0.01:
            trajectory.append(pos)
        yaw = get_robot_yaw(compass.getValues())
        dists = [s.getValue() for s in ps]

        if get_distance(pos, goal) < GOAL_RADIUS:
            print(f"SUCCESS: Goal {goal_idx + 1} reached at {pos}!")
            goal_idx += 1
            
            if goal_idx >= len(goals):
                print("MISSION COMPLETE: All goals reached.")
                lm.setVelocity(0); rm.setVelocity(0)
                plt.figure(figsize=(10, 8))
                xs, ys = zip(*trajectory)
                plt.plot(xs, ys, marker='.', markersize=2, linestyle='-', color='blue', label='Robot Trajectory')
                plt.plot([m_line_start[0], goals[0][0]], [m_line_start[1], goals[0][1]], color='gray', linestyle='--', label='M-line (Goal 1)')
                if hit_points_log:
                    h_xs, h_ys = zip(*hit_points_log)
                    plt.scatter(h_xs, h_ys, color='red', s=150, marker='X', zorder=5, label='Hit Point')
                if leave_points_log:
                    l_xs, l_ys = zip(*leave_points_log)
                    plt.scatter(l_xs, l_ys, color='cyan', s=150, marker='o', zorder=5, label='Leave Point')
                plt.scatter(trajectory[0][0], trajectory[0][1], color='green', s=150, zorder=5, label='Start')
                for i, g in enumerate(goals):
                    plt.scatter(g[0], g[1], color='magenta', marker='*', s=200, zorder=5, label=f'Goal {i+1}')
                plt.title('Robot Trajectory (Hybrid Bug2 + Wall-Following)')
                plt.xlabel('X Coordinate'); plt.ylabel('Y Coordinate')
                plt.legend(); plt.grid(True); plt.axis('equal'); plt.show()
                break

            goal = goals[goal_idx]
            print(f"Transitioning to Goal {goal_idx + 1} at {goal}.")

            if goal_idx == 1:
                print("Switching to LEFT-WALL FOLLOWING algorithm.")
                print("Executing 90-degree RIGHT turn...")
                initial_yaw = get_robot_yaw(compass.getValues())
                target_yaw = initial_yaw - (math.pi / 2.0)
                target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
                
                lm.setVelocity(0.5 * MAX_SPEED)
                rm.setVelocity(-0.5 * MAX_SPEED)
                
                while robot.step(timestep) != -1:
                    current_yaw = get_robot_yaw(compass.getValues())
                    angle_diff = target_yaw - current_yaw
                    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
                    
                    if abs(angle_diff) < 0.05:
                        print("Turn complete.")
                        lm.setVelocity(0)
                        rm.setVelocity(0)
                        break
                
                print("Stabilizing with a short forward movement...")
                lm.setVelocity(MAX_SPEED * 0.5)
                rm.setVelocity(MAX_SPEED * 0.5)
                for _ in range(10):
                    robot.step(timestep)
                
                print("Now following left wall.")


            state = 'GOAL_SEEKING'
            hit_point = None
            dist_at_hit = float('inf')
            m_line_start = pos
            continue

        ls, rs = 0, 0

        
        if goal_idx == 0:
            front_obstacle = dists[0] > WALL_THRESHOLDS['obstacle'] or dists[7] > WALL_THRESHOLDS['obstacle']
            if state == 'GOAL_SEEKING':
                if front_obstacle:
                    state = 'WALL_FOLLOWING'
                    hit_point, dist_at_hit = pos, get_distance(pos, goal)
                    hit_points_log.append(hit_point)
                    print(f"Obstacle detected! To WALL_FOLLOWING. Dist: {dist_at_hit:.2f}")
                    ls, rs = follow_right_wall(dists, WALL_THRESHOLDS, WALL_SPEEDS)
                else: 
                    angle_to_goal = get_bearing_to_goal(pos, goal)
                    angle_diff = (angle_to_goal - yaw + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff) > 0.1:
                        turn_speed = 0.8 * MAX_SPEED
                        ls, rs = -math.copysign(turn_speed, angle_diff), math.copysign(turn_speed, angle_diff)
                    else:
                        ls, rs = MAX_SPEED, MAX_SPEED
            elif state == 'WALL_FOLLOWING':
                current_dist = get_distance(pos, goal)
                if (is_on_m_line(pos, m_line_start, goal) and
                    current_dist < dist_at_hit - EXIT_MARGIN and
                    get_distance(pos, hit_point) > HIT_PROXIMITY_THR):
                    state = 'GOAL_SEEKING'
                    leave_points_log.append(pos)
                    print(f"Back on M-line. To GOAL_SEEKING. Dist: {current_dist:.2f}")
                    hit_point = None
                    continue
                
                ls, rs = follow_right_wall(dists, WALL_THRESHOLDS, WALL_SPEEDS)
        
        elif goal_idx == 1:
            print("IM HERE")
            ls, rs = follow_left_wall(dists, WALL_THRESHOLDS, WALL_SPEEDS)

        lm.setVelocity(ls)
        rm.setVelocity(rs)
