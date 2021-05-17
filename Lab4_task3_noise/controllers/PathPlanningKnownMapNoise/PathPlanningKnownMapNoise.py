"""PathPlanningKnownMapNoise controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import random
from collections import deque

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#enable distance sensors
fds = robot.getDevice('front_ds')
lds = robot.getDevice('left_ds')
rds = robot.getDevice('right_ds')
fds.enable(timestep)
lds.enable(timestep)
rds.enable(timestep)

# getting the position sensors
lps = robot.getDevice('left wheel sensor')
rps = robot.getDevice('right wheel sensor')
lps.enable(timestep)
rps.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# values for robot
wheel_radius = 1.6 / 2.0
wheel_circ = 2 * 3.14 * wheel_radius
enc_unit = wheel_circ / 6.28
max_speed = 4
# distance between wheels in inches and middle of the robot d_mid
d = 2.28
d_mid = d / 2.0

# world values
# n is index+1 of the associated (row,column) combinations below
# for example, (0,0) is row=0, column=0, so n would be index of 
# this in the list, which is 0, so add 1, so n=0+1=1
n_rc = [(0,0), (0,1), (0,2), (0,3),
        (1,0), (1,1), (1,2), (1,3),
        (2,0), (2,1), (2,2), (2,3),
        (3,0), (3,1), (3,2), (3,3)
       ]

# state of robot
dir = "North"
# new position to keep track of x,y as the robot keeps moving
new_pos = [15.0, -15.0]
# keep track of the last values from position sensors, so the difference can be added to position
last_vals = [0, 0]
# robot pose to hold x, y, grid number n, and orientation theta(represented as q)
robot_pose = [15.0, -15.0, 16, 180]

# Maze configuration, if any of the WNES are set to 1, there
# is either an inner or outer wall there
#[Visited, West, North, East, South], only external walls configured at start
grid_maze = [[0, 1, 1, 0, 1], [0, 0, 1, 0, 1], [0, 0, 1, 0, 0], [0, 0, 1, 1, 0],
             [0, 1, 1, 0, 0], [0, 0, 1, 1, 0], [0, 1, 0, 1, 0], [0, 1, 0, 1, 0], 
             [0, 1, 0, 1, 0], [0, 1, 0, 0, 1], [0, 0, 0, 1, 1], [0, 1, 0, 1, 0],
             [0, 1, 0, 0, 1], [0, 0, 1, 0, 1], [0, 0, 1, 0, 1], [0, 0, 0, 1, 1]]

# array to keep track of all non-visited and visited cells, '.'=non-visited, and 'X'=visited
visited_cells = ['.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', 'X']  
                 
# number of cells in the world
num_cells = 16

# goal pose to end at, in format [cell, facing_direction]
goal_pose = [6, "West"]        
                 
                 
# converts meters to inches
def m_to_i(meters):
    return meters * 39.3701


def pos_s_to_inches(val):
    return math.fabs(val * wheel_radius)
        

def print_measurements():
    print(f"left: {pos_s_to_inches(lps.getValue())}, right: {pos_s_to_inches(rps.getValue())}, imu: {(imu.getRollPitchYaw()[2] * 180) / 3.14159}")
    

def get_p_sensors_vals():
    # returns left and right position sensors
    return pos_s_to_inches(lps.getValue()), pos_s_to_inches(rps.getValue())

    
def get_d_sensors_vals():
    # returns left, front, right sensors in inches
    return m_to_i(lds.getValue()), m_to_i(fds.getValue()), m_to_i(rds.getValue())


def get_time(distance, speed):
    return distance / speed


def move(inches, timestep):
    seconds = get_time(inches, max_speed)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # update the robot
        update_robot()
        #if dest != None:
        #    print(f"Moving towards cell: {dest}...")
        #print(f"Moving {inches} inches forward...")
        # get the walls around the robot and add them to the map
        if robot.getTime() < end_time:
            leftMotor.setVelocity(max_speed/wheel_radius)
            rightMotor.setVelocity(max_speed/wheel_radius)
        else:
            stop_motors()
            break


def stop_motors():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    print("Motors stopped.")


def get_rot_speed_rad(degrees, seconds):
    circle = d_mid * 2 * math.pi
    dist = (degrees / 360) * circle
    linear_vel = dist / seconds
    left_wheel_speed = linear_vel / wheel_radius
    right_wheel_speed = -1 * linear_vel / wheel_radius
    return left_wheel_speed, right_wheel_speed

  
def rotate(degrees, seconds, timestep, direction):
    global last_vals
    # get the left and right rotaional speeds to turn x degrees in y seconds
    left, right = get_rot_speed_rad(degrees, seconds)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # update and print the robot's details
        update_robot(rotating=True)
        # still update the last vals
        vals = get_p_sensors_vals()
        last_vals = vals
        print(f"Rotating {direction}...")
        if robot.getTime() < end_time:
            leftMotor.setVelocity(left)
            rightMotor.setVelocity(right)
        else:
            stop_motors()
            break    

            
def turn_left(ts, degrees=-90.5):
    rotate(degrees, 1.5, ts, "left")           
            
            
def turn_right(ts, degrees=90.5):
    rotate(degrees, 1.5, ts, "right")
    

def check_if_robot_should_stop():
    # go through all the cells to see if it has a 0,
    # a cell that has not been visited yet, in the list or not
    for el in grid_maze:
        # if there is even a single 0 in the list, there is a cell
        # that hasn't been visited to yet
        if el[0] == 0:
            # there is a cell yet to be visited, 
            # robot should keep going, don't stop
            return False
    # all the cells were 1's at this point, so all cells have been 
    # visited, robot should stop
    return True      


def update_robot(rotating=False):
    # Indicated as (x,y,n,q), where “x,y” represents
    # the robot position in global coordinates, “n” represents 
    # the grid cell number, and “q ” represents the robot global
    # orientation with respect to the global frame of reference,
    # e.g. Pose = (11.5, 2.3, 8, 1.1).
    global robot_pose
    
    # print the time
    print(80*"-")
    print(f"Time: {robot.getTime()}")
    
    # get the new position sensor measurements
    vals = get_p_sensors_vals()
    
    if not rotating:
        x, y = get_robot_x_y(vals)
        n = get_current_grid_cell(x, y)
    else:
        x = robot_pose[0]
        y = robot_pose[1]
        n = robot_pose[2]
    q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
        
    # update the robot's pose
    robot_pose = [x, y, n, q]
        
    # print new robot pose
    print(f"Pose: {robot_pose}")
        
    # update the direction the robot is facing and print it
    update_direction()
    print(f"Robot current grid cell: {robot_pose[2]}, Direction: {dir}")
        
    # print the visited cells
    print_visited_cells()
        
    # print the updated grid maze map
    print_maze(grid_maze)


def get_robot_x_y(vals):
    global new_pos
    global last_vals
    diff = [vals[0] - last_vals[0], vals[1] - last_vals[1]]
    for i in range(len(diff)):
        diff[i] = math.fabs(diff[i])
    
    if math.fabs(diff[0]) >= .3 or math.fabs(diff[1]) >= .3:
            diff[0] = 0.3
            diff[1] = 0.3
            
    # diff average of the left and right wheel
    diff_avg = (diff[0]+ diff[1]) / 2.0
    
    # x and y are dependent on the direction the robot is moving in
    if dir == "North":
        x = new_pos[0]
        y = new_pos[1] + diff_avg
    elif dir == "West":
        x = new_pos[0] - diff_avg
        y = new_pos[1] 
    elif dir == "East":
        x = new_pos[0] + diff_avg
        y = new_pos[1] 
    elif dir == "South":
        x = new_pos[0] 
        y = new_pos[1] - diff_avg
    # update the last vals
    last_vals = vals
    # store the new x and y into the new_pos
    new_pos = x, y
    return x, y


def get_current_grid_cell(x, y):
    n = 0
    row = 0
    col = 0
    
    # how to determine grid row from y
    if y >= -20 and y < -10:
        # the bottom row
        row = 3
    elif y >= -10 and y < 0:
        # row above the bottom row
        row = 2
    elif y >= 0 and y < 10:
        # row beneath the top row
        row = 1 
    elif y >= 10 and y <= 20:
        # the top row
        row = 0 
        
    # how to determine grid column from x
    if x >= -20 and x <= -10:
        # the left column
        col = 0
    elif x > -10 and x <= 0:
        # the middle left column
        col = 1
    elif x > 0 and x <= 10:
        # the middle right column
        col = 2 
    elif x > 10 and x <= 20:
        # the right column
        col = 3 
        
    # get the n based off of the row, column combination
    for i in range(len(n_rc)):
        if n_rc[i][0] == row and n_rc[i][1] == col:
            # grid cell is 1 plus index of row,col combination
            n = i + 1
            break
    return n
    

def update_direction():
    global dir
    dir = get_direction(robot_pose[3]) 


def get_direction(imu_val):
    if (imu_val <= -135 and imu_val >= -180) or (135 <= imu_val <= 180):
        dir = "North"
    elif imu_val <= -45 and imu_val > -135:
        dir = "West"
    elif 45 <= imu_val <= 135:
        dir = "East"
    elif (-45 < imu_val <= 0) or (0 <= imu_val < 45):
        dir = "South"
    return dir 


def get_available_turns(walls):
    # a 0 for a wall means there is no wall in that direction
    # available_turns also returned in order [left, front, right]
    # 0, means not available, 1 means available 
    # count is the number of available turns
    count = 0
    available = [0, 0, 0]
    for i in range(len(walls)):
        if walls[i] == 0:
            available[i] = 1
            count += 1
        # otherwise if a wall is there, walls[i] == 1, leave it as a 0
    return available, count


def print_visited_cells():
    pr_vc = ""
    for i in range(len(visited_cells)):
        if i % 4 == 0 and i != 0:
            pr_vc += "\n"
        pr_vc += visited_cells[i]
    print("Visited cells:")
    print(pr_vc)


def mark_cell_visited(cell):
    global grid_maze
    global visited_cells
    grid_maze[cell-1][0] = 1
    visited_cells[cell-1] = 'X'


def face_north(ts):
    global dir
    while robot.step(ts) != -1:
        if dir == "West":
            # turn right to face north again
            turn_right(ts)
        elif dir == "South":
            # turn left twice to face north again
            turn_left(ts)
            turn_left(ts)
        elif dir == "East":
            # turn left to face north again
            turn_left(ts)
        if dir == "North":
            break
    dir = "North"


def face_dir(ts, direction="North"):
    # get the destination angle for the direction
    if direction == "North":
        dir_angle = 180.0
    elif direction == "East":
        dir_angle = 90.0
    elif direction == "South":
        dir_angle = 0.0
    elif direction == "West":
        dir_angle = -90.0

    speed = 2
    tolerance = 0.1
    while robot.step(ts) != -1:
        q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
        diff = dir_angle - q
        abs_diff = math.fabs(diff)
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        print(f"q: {q}")
        print(f"diff: {diff}")
        print(f"abs_diff: {abs_diff}")
        print(f"Direction: {get_direction(q)}")
        print(f"Rotating until facing: {direction}..")
        print(f"speed: {speed}")
        
        # slow the speed down, or flip direction based on abs_diff 
        if diff < 0 or (180 <= abs_diff <= 360):
            if abs_diff < 0.2:
                speed = -0.05
            elif abs_diff < 5:
                speed = -0.2
            else:
                speed = -2.0
        elif abs_diff < 0.2:
            speed = 0.05
        elif abs_diff < 5:
            speed = 0.2 
        elif abs_diff < 180:
            speed = 2.0
            
        # if the tolerance has been met, stop
        if abs_diff < tolerance:
            stop_motors()
            break
        else:
            leftMotor.setVelocity(-1 * speed)
            rightMotor.setVelocity(speed)   


# changed from LAB 3: VNESW to LAB 4: VWNES
def print_maze(maze):
    print("________________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        elif (maze[x][0] == 1):
            v1 = "V"
        else:
            v1 = str(maze[x][0])
            
        if (maze[x+1][0] == 0):
            v2 = "?"
        elif (maze[x+1][0] == 1):
            v2 = "V"
        else:
            v2 = str(maze[x+1][0])
            
        if (maze[x+2][0] == 0):
            v3 = "?"
        elif (maze[x+2][0] == 1):
            v3 = "V"
        else:
            v3 = str(maze[x+2][0])
            
        if (maze[x+3][0] == 0):
            v4 = "?"
        elif (maze[x+3][0] == 1):
            v4 = "V"
        else:
            v4 = str(maze[x+3][0])
         
        print("|  "+ str(maze[x][2]) +"\t  " +str(maze[x+1][2])+"\t  " +str(maze[x+2][2])
              +"\t  " +str(maze[x+3][2])+ "    |")
              
        print("|" +str(maze[x][1]) + " " +v1+" " + str(maze[x][3])+"\t" +str(maze[x+1][1])+ " " +v2+" " + str(maze[x+1][3])
              +"\t" +str(maze[x+2][1])+ " " +v3+" " + str(maze[x+2][3])
              +"\t" +str(maze[x+3][1]) + " " +v4+" " + str(maze[x+3][3]) +"  |")
              
        print("|  "+str(maze[x][4]) +"\t  " +str(maze[x+1][4])+"\t  " +str(maze[x+2][4])
              +"\t  " +str(maze[x+3][4])+"    |")
              
        if(i==3):
            print("|_______________________________________|\n")
        else:
            print("|                                       |")


def wave_front(ts, goal_cell):
    # get the index for the starting and ending cell
    goal_cell_ind = goal_cell - 1
    start_cell_ind = robot_pose[2] - 1
    
    # mark the goal cell
    grid_maze[goal_cell_ind][0] = 2
    
    # set the wave count to start at the next value from the goal
    count = grid_maze[goal_cell_ind][0] + 1
    
    print("Start wave front:")
    print_maze(grid_maze)
    
    
    while robot.step(ts) != -1:
        # print the time
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        
        # print the wave count 
        print(f"Wave count: {count}")
        
        # get the cells to get the walls of 
        cells = []
        for i in range(num_cells):
            # if the cell has the previous wave count's value,
            # get that cell so we can add the current wave count to the grid_maze
            if grid_maze[i][0] == count - 1:
                cells.append(i)
        print(f"Cells: {cells}")
        
        # for every cell index, get the walls and then add the
        # wave count value to the grid maze
        for c in cells:
            # get the walls surrounding the current cell
            walls = grid_maze[c]
            print(f"Cell: {c}, Walls: {walls}")
            
            if walls[1] == 0 and grid_maze[c-1][0] == 0:
                grid_maze[c-1][0] = count
            if walls[2] == 0 and grid_maze[c-4][0] == 0:
                grid_maze[c-4][0] = count
            if walls[3] == 0 and grid_maze[c+1][0] == 0:
                grid_maze[c+1][0] = count
            if walls[4] == 0 and grid_maze[c+4][0] == 0:
                grid_maze[c+4][0] = count
        
        # print the grid_maze to see the alterations
        print_maze(grid_maze)
        
        # if the starting cell equals the value of the wave count,
        # it was just reached
        if grid_maze[start_cell_ind][0] == count:
            break
            
        # increment the wave count
        count += 1


def plan_path(ts, goal_cell):
    # stack for path planning
    plan = deque() 
    
    # get the index for the starting and ending cell
    goal_cell_ind = goal_cell - 1
    start_cell_ind = robot_pose[2] - 1
    
    # starting cell's value
    start_val = grid_maze[start_cell_ind][0]
    
    # current value
    current_val = start_val - 1
    
    # the index of the cell currently being examined for its neighbors
    current_cell = start_cell_ind
    
    while robot.step(ts) != -1:
        # the move to go from the current cell to the next cell
        mov = ""
    
        # print the time
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        
        # get the walls surrounding the current cell
        # walls are [V, W, N, E, S]
        walls = grid_maze[current_cell]
        
        #print(f"current cell: {current_cell}")
        
        # check the cells around for the next value
        if current_cell > 3:
            if grid_maze[current_cell - 4][0] == current_val and walls[2] == 0:
                mov = "forward"
        if current_cell >0 :
            if grid_maze[current_cell - 1][0] == current_val and walls[1] == 0:
                mov = "left"
        if current_cell < 15:
            if grid_maze[current_cell + 1][0] == current_val and walls[3] == 0:
                mov = "right"
        if current_cell < 11:
            if grid_maze[current_cell + 4][0] == current_val and walls[4] == 0:
                mov = "down"   
        
        # append the move to the stack
        plan.append((mov, current_val+1, current_cell))
        
        # update the current cell based on the move
        if mov == "forward":
            current_cell -= 4
        elif mov == "left":
            current_cell -= 1
        elif mov == "right":
            current_cell += 1
        elif mov == "down":
            current_cell += 4
            
        
        # print the stack information
        #print(f"number of moves in plan so far: {len(plan)}")
        print(f"plan of moves: {plan}")
        
        # update the current_val
        current_val -= 1
        
        # if on the goal cell, stop planning
        if current_cell == goal_cell_ind:
            break
            
    # return the plan once the moves have been made from start to goal
    return plan
    
    
def execute_plan(ts, plan):
    while robot.step(ts) != -1:
        # make the robot face north if it isn't already
        if dir != "North":
            face_north(ts)
        
        # pop the left most move in the plan deque to get the current move to make
        mov = plan.popleft()
        
        # print the move and get the movement direction
        #print(f"current move: {mov}")
        m = mov[0]
        
        # if the robot is facing north
        if dir == "North":
            # use the move m to turn the robot accordingly
            if m == "left":
                turn_left(ts)
            elif m == "right":
                turn_right(ts)
            elif m == "down":
                turn_right(ts)
                turn_right(ts)
                
            # once the robot has been aligned according to m, 
            # move it to the next cell according to the wave front plan
            move(10.0, ts)
            visited_cells[robot_pose[2]-1] = "X"
        
        # if the robot is at the goal cell
        if robot_pose[2] == goal_pose[0]:
            # make the robot face north
            face_north(ts)
            
            # make the robot turn according to the direction in the goal_pose
            if goal_pose[1] == "West":
                turn_left(ts)
            elif goal_pose[1] == "East":
                turn_right(ts)
            elif goal_pose[1] == "West":
                turn_right(ts)
                turn_right(ts)
                
            # see if the robot aligned to the goal pose's direction
            if dir == goal_pose[1]:
                break  
    return 0        

def main():
    if goal_pose[0] < 1:
        goal_pose[0] = 1
    elif goal_pose[0] > 16:
        goal_pose[0] = 16
    
    while robot.step(timestep) != -1:
        # alter the grid_maze so that it can be used to make the path
        wave_front(timestep, goal_pose[0])
        
        # make the plan from the altered grid_maze
        plan = plan_path(timestep, goal_pose[0])
        
        # execute the plan
        val = execute_plan(timestep, plan)
        
        # print the visited cells
        print_visited_cells()
        
        # if val ='s 0, executed successfully
        if val == 0:
            print(80*"-")
            print(f"Time: {robot.getTime()}")
            print("GOAL")
            break       


if __name__ == "__main__":
    main()
