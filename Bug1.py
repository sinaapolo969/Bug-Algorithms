from controller import Robot
import math
import matplotlib.pyplot as plt


def get_bearing_to_goal(pos1, pos2):
    return math.atan2(pos2[1] - pos1[1], pos2[0] - pos1[0])

def get_robot_yaw(compass_values):
    return math.atan2(compass_values[0], compass_values[1])

def get_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def follow_right_wall(dists, thresholds, speeds):
    """Implements robust right-hand wall-following logic for the first leg."""
    front_obstacle = dists[0] > thresholds['obstacle'] or dists[7] > thresholds['obstacle']
    right_wall_too_close = dists[2] > thresholds['too_close'] or dists[1] > thresholds['too_close']
    right_wall_missing = dists[2] < thresholds['obstacle'] and dists[1] < thresholds['obstacle']

    if front_obstacle: return -speeds['turn'], speeds['turn']
    elif right_wall_too_close: return speeds['adjust'], speeds['straight']
    elif right_wall_missing: return speeds['straight'], speeds['adjust']
    else: return speeds['straight'], speeds['straight']

def follow_left_wall(dists, const):
    """
    Implements left-wall-following logic for the second leg.
    """
    front_obstacle = dists[0] > const['OBSTACLE_THR'] or dists[7] > const['OBSTACLE_THR']
    left_wall_too_close = dists[5] > const['WALL_TOO_CLOSE'] or dists[6] > const['WALL_TOO_CLOSE']
    left_wall_missing = dists[5] < const['OBSTACLE_THR'] and dists[6] < const['OBSTACLE_THR']

    if front_obstacle:
        return 0.8 * const['MAX_SPEED'], -0.8 * const['MAX_SPEED']
    elif left_wall_too_close:
        return const['MAX_SPEED'], 0.7 * const['MAX_SPEED']
    elif left_wall_missing:
        return 0.7 * const['MAX_SPEED'], const['MAX_SPEED']
    else:
        return const['MAX_SPEED'], const['MAX_SPEED']

if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    goals = [(0, 4.7), (-3.32, 3.75)]
    goal_idx = 0
    goal = goals[goal_idx]
    
    state = 'GOAL_SEEKING'
    wall_side = 'RIGHT' 
    hit_point, closest_point_on_obstacle = None, None
    min_dist_to_goal_on_obstacle = float('inf')
    circumnavigation_started = False
    leave_obstacle_counter = 0

    trajectory, hit_points_log, closest_points_log = [], [], []

    CONSTANTS = {
        'MAX_SPEED': 6.28,
        'GOAL_RADIUS': 0.1,
        'LEAVE_OBSTACLE_STEPS': 20,
        'OBSTACLE_THR': 90.0,
        'WALL_TOO_CLOSE': 120.0
    }
    WALL_SPEEDS = {'turn': 0.8 * CONSTANTS['MAX_SPEED'], 'straight': CONSTANTS['MAX_SPEED'], 'adjust': 0.7 * CONSTANTS['MAX_SPEED']}
    WALL_THRESHOLDS = {'obstacle': CONSTANTS['OBSTACLE_THR'], 'too_close': CONSTANTS['WALL_TOO_CLOSE']}

    gps = robot.getDevice('gps'); gps.enable(timestep)
    compass = robot.getDevice('compass'); compass.enable(timestep)
    ps = [robot.getDevice(f'ps{i}') for i in range(8)]; [p.enable(timestep) for p in ps]
    lm = robot.getDevice('left wheel motor'); rm = robot.getDevice('right wheel motor')
    lm.setPosition(float('inf')); rm.setPosition(float('inf'))
    lm.setVelocity(0); rm.setVelocity(0)

    robot.step(timestep)
    print(f"Starting Mission. Initial Algorithm: Bug1. Initial Goal: {goal}.")

    while robot.step(timestep) != -1:
        gps_pos = gps.getValues()
        pos = (gps_pos[0], gps_pos[1])
        trajectory.append(pos)
        yaw = get_robot_yaw(compass.getValues())
        dists = [s.getValue() for s in ps]
        
        if get_distance(pos, goal) < CONSTANTS['GOAL_RADIUS']:
            print(f"SUCCESS: Goal {goal_idx + 1} reached at {pos}!")
            goal_idx += 1
            
            if goal_idx >= len(goals):
                print("MISSION COMPLETE: All goals reached.")
                lm.setVelocity(0); rm.setVelocity(0)
                plt.figure(figsize=(10, 8))
                xs, ys = zip(*trajectory)
                plt.plot(xs, ys, 'c-', label='Trajectory', zorder=1)
                plt.scatter(trajectory[0][0], trajectory[0][1], color='blue', s=150, marker='s', label='Start Point', zorder=2)
                for i, g in enumerate(goals):
                    plt.scatter(g[0], g[1], color='lime', s=200, marker='P', label=f'Goal {i+1}', zorder=3)
                if hit_points_log:
                    h_xs, h_ys = zip(*hit_points_log)
                    plt.scatter(h_xs, h_ys, color='red', s=150, marker='X', label='Hit Point (H)', zorder=4)
                if closest_points_log:
                    c_xs, c_ys = zip(*closest_points_log)
                    plt.scatter(c_xs, c_ys, color='green', s=200, marker='*', label='Leave Point (P)', zorder=5)
                plt.title('Robot Trajectory'); plt.xlabel('X Position (m)'); plt.ylabel('Y Position (m)')
                plt.legend(); plt.grid(True); plt.gca().set_aspect('equal', adjustable='box'); plt.show()
                break
            
            goal = goals[goal_idx]
            print(f"Transitioning to Goal {goal_idx + 1} at {goal}.")

            if goal_idx == 1:
                print("Switching to LEFT-WALL FOLLOWING algorithm for the second leg.")
                print("Executing 90-degree RIGHT turn to face the wall...")
                initial_yaw = get_robot_yaw(compass.getValues())
                target_yaw = initial_yaw - (math.pi / 2.0)
                
                target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

                lm.setVelocity(0.4 * CONSTANTS['MAX_SPEED'])
                rm.setVelocity(-0.4 * CONSTANTS['MAX_SPEED'])
                
                while robot.step(timestep) != -1:
                    current_yaw = get_robot_yaw(compass.getValues())
                    angle_diff = target_yaw - current_yaw
                    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff) < 0.05:
                        break
                
                wall_side = 'LEFT'
                print(f"Turn complete. Robot is now permanently following the {wall_side} wall.")

            state = 'GOAL_SEEKING' 
            hit_point, closest_point_on_obstacle = None, None
            min_dist_to_goal_on_obstacle = float('inf')
            circumnavigation_started = False
            continue

        ls, rs = 0, 0
        

        if goal_idx == 0:
            front_obstacle = dists[0] > CONSTANTS['OBSTACLE_THR'] or dists[7] > CONSTANTS['OBSTACLE_THR']
            
            if leave_obstacle_counter > 0:
                leave_obstacle_counter -= 1
                state = 'GOAL_SEEKING'
            
            if state == 'GOAL_SEEKING':
                if front_obstacle:
                    state = 'WALL_FOLLOWING'
                    hit_point, closest_point_on_obstacle = pos, pos
                    min_dist_to_goal_on_obstacle = get_distance(pos, goal)
                    circumnavigation_started = False
                    hit_points_log.append(hit_point)
                    print(f"Hit obstacle. Entering {wall_side} WALL_FOLLOWING.")
                else:
                    angle_to_goal = get_bearing_to_goal(pos, goal)
                    angle_diff = (angle_to_goal - yaw + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff) > 0.1:
                        turn_speed = 0.8 * CONSTANTS['MAX_SPEED']
                        ls, rs = -math.copysign(turn_speed, angle_diff), math.copysign(turn_speed, angle_diff)
                    else:
                        ls, rs = CONSTANTS['MAX_SPEED'], CONSTANTS['MAX_SPEED']

            elif state == 'WALL_FOLLOWING':
                current_dist_to_goal = get_distance(pos, goal)
                if current_dist_to_goal < min_dist_to_goal_on_obstacle:
                    min_dist_to_goal_on_obstacle = current_dist_to_goal
                    closest_point_on_obstacle = pos
                
                if not circumnavigation_started and get_distance(pos, hit_point) > 0.15:
                    circumnavigation_started = True
                
                if circumnavigation_started and get_distance(pos, hit_point) < CONSTANTS['GOAL_RADIUS']:
                    state = 'RETURNING_TO_CLOSEST_POINT'
                    closest_points_log.append(closest_point_on_obstacle)
                    print(f"Circumnavigation complete. Closest point is {closest_point_on_obstacle}.")
            
            elif state == 'RETURNING_TO_CLOSEST_POINT':
                if get_distance(pos, closest_point_on_obstacle) < CONSTANTS['GOAL_RADIUS'] * 1.5:
                    print(f"Reached closest point. Leaving obstacle.")
                    leave_obstacle_counter = CONSTANTS['LEAVE_OBSTACLE_STEPS']
                    continue

            if state in ['WALL_FOLLOWING', 'RETURNING_TO_CLOSEST_POINT']:
                ls, rs = follow_right_wall(dists, WALL_THRESHOLDS, WALL_SPEEDS)
        
        else: 
            ls, rs = follow_left_wall(dists, CONSTANTS)

        lm.setVelocity(ls)
        rm.setVelocity(rs)