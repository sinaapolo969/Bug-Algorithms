from controller import Robot
import math
import matplotlib.pyplot as plt

def get_bearing_to_goal(pos1, pos2):
    return math.atan2(pos2[1] - pos1[1], pos2[0] - pos1[0])

def get_robot_yaw(compass_values):
    return math.atan2(compass_values[0], compass_values[1])

def get_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    goals = [(0, 4.7), (-3, 4.07)]
    goal_idx = 0
    goal = goals[goal_idx]

    state = 'GOAL_SEEKING'
    
    trajectory = []
    hit_points_log = []
    leave_points_log = []

    MAX_SPEED = 6.28
    GOAL_RADIUS = 0.1
    OBSTACLE_THR = 90.0
    WALL_TOO_CLOSE = 120.0

    gps = robot.getDevice('gps'); gps.enable(timestep)
    compass = robot.getDevice('compass'); compass.enable(timestep)
    ps = [robot.getDevice(f'ps{i}') for i in range(8)]
    for p in ps: p.enable(timestep)
    lm = robot.getDevice('left wheel motor'); rm = robot.getDevice('right wheel motor')
    lm.setPosition(float('inf')); rm.setPosition(float('inf'))
    lm.setVelocity(0); rm.setVelocity(0)

    robot.step(timestep)

    while robot.step(timestep) != -1:
        gps_pos = gps.getValues()
        pos = (gps_pos[0], gps_pos[1])
        trajectory.append(pos)

        yaw = get_robot_yaw(compass.getValues())
        dists = [s.getValue() for s in ps]
        
        front_obstacle = dists[0] > OBSTACLE_THR or dists[7] > OBSTACLE_THR
        
        left_wall = dists[5] > OBSTACLE_THR
        left_corner = dists[6] > OBSTACLE_THR
        left_wall_too_close = dists[5] > WALL_TOO_CLOSE or dists[6] > WALL_TOO_CLOSE
        
        if get_distance(pos, goal) < GOAL_RADIUS:
            print(f"Goal {goal_idx + 1} reached!!")
            goal_idx += 1
            if goal_idx >= len(goals):
                print("Mission complete.")
                lm.setVelocity(0)
                rm.setVelocity(0)

                print("Generating final trajectory plot...")
                plt.figure(figsize=(10, 8))
                if trajectory:
                    xs, ys = zip(*trajectory)
                    plt.plot(xs, ys, 'c-', label='Trajectory', zorder=1)
                    plt.scatter(trajectory[0][0], trajectory[0][1], color='blue', s=150, marker='s', label='Start Point', zorder=2)
                for i, g in enumerate(goals):
                    plt.scatter(g[0], g[1], color='lime', s=200, marker='P', label=f'Goal {i+1}', zorder=3)
                if hit_points_log:
                    h_xs, h_ys = zip(*hit_points_log)
                    plt.scatter(h_xs, h_ys, color='red', s=150, marker='X', label='Hit Point', zorder=4)
                if leave_points_log:
                    l_xs, l_ys = zip(*leave_points_log)
                    plt.scatter(l_xs, l_ys, color='green', s=100, marker='o', label='Leave Point', zorder=4)
                plt.title('Robot Trajectory (Bug Algorithm)')
                plt.xlabel('X Position (m)'), plt.ylabel('Y Position (m)')
                plt.legend(), plt.grid(True), plt.gca().set_aspect('equal', adjustable='box')
                plt.show()
                
                while robot.step(timestep) != -1: pass
                break

            goal = goals[goal_idx]
            
            if goal_idx == 1:
                print("First goal reached. Executing 90-degree right turn.")
                initial_yaw = get_robot_yaw(compass.getValues())
                target_yaw = initial_yaw - (math.pi / 2.0)

                lm.setVelocity(0.4 * MAX_SPEED)
                rm.setVelocity(-0.4 * MAX_SPEED)
                
                while robot.step(timestep) != -1:
                    current_yaw = get_robot_yaw(compass.getValues())
                    angle_diff_to_target = (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff_to_target) < 0.05: # Small tolerance for completion
                        break

                print("Turn complete. Initiating left-wall-following.")
                state = 'WALL_FOLLOWING'
            else:
                state = 'GOAL_SEEKING'
            
            continue

        angle_to_goal = get_bearing_to_goal(pos, goal)
        angle_diff = (angle_to_goal - yaw + math.pi) % (2 * math.pi) - math.pi
        
        ls, rs = 0, 0 

        if state == 'GOAL_SEEKING':
            if front_obstacle:
                state = 'WALL_FOLLOWING'
                hit_points_log.append(pos) 
                print("Obstacle hit. Starting wall-following mode.")
                ls, rs = 0.5 * MAX_SPEED, -0.5 * MAX_SPEED 
            else:
                if abs(angle_diff) > 0.1:
                    turn_speed = 0.8 * MAX_SPEED
                    ls = -math.copysign(turn_speed, angle_diff)
                    rs = math.copysign(turn_speed, angle_diff)
                else:
                    ls = rs = MAX_SPEED
        
        else:
            if not front_obstacle and abs(angle_diff) < 0.2:
                state = 'GOAL_SEEKING'
                leave_points_log.append(pos) 
                print("Path to goal is clear. Returning to goal-seeking mode.")
                continue
            
            if front_obstacle:
                print("LWF: Obstacle front, turning right.")
                ls, rs = 0.8 * MAX_SPEED, -0.8 * MAX_SPEED
            elif left_wall_too_close:
                print("LWF: Too close to left wall, turning right.")
                ls, rs = MAX_SPEED, 0.7 * MAX_SPEED
            elif not left_wall and not left_corner:
                print("LWF: Lost left wall, turning left.")
                ls, rs = 0.7 * MAX_SPEED, MAX_SPEED
            else:
                print("LWF: Following wall, going straight.")
                ls = rs = MAX_SPEED

        lm.setVelocity(ls)
        rm.setVelocity(rs)