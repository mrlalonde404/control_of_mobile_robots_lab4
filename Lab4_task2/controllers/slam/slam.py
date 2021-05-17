"""slam controller."""

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
new_pos = [0, 0]
# keep track of the last values from position sensors, so the difference can be added to position
last_vals = [0, 0]
# robot pose to hold x, y, grid number n, and orientation theta(represented as q)
robot_pose = [0.0, 0.0, 0, 180]

# global boolean, localized, while the robot has not been localized, dont output x,y pose information
localized = False

# Maze configuration, if any of the WNES are set to 1, there
# is either an inner or outer wall there
#[Visited, West, North, East, South], only external walls configured at start
grid_maze = [[0, 1, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 1, 1, 0],
             [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0], 
             [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0],
             [0, 1, 0, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 1, 1]]

# array to keep track of all non-visited and visited cells, '.'=non-visited, and 'X'=visited
visited_cells = ['.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.']  
                 
# number of cells in the world
num_cells = 16

# hardcoded cylinders, x, y, and radiuses all of 3.14
y_cyl = (-20.0, 20, 3.14) # yellow cylinder
r_cyl = (20.0, 20, 3.14) # red cylinder
g_cyl = (-20.0, -20, 3.14) # green cylinder
b_cyl = (20.0, -20, 3.14) # blue cylinder      

# colors
Y = [1.0, 1.0, 0.0]
R = [1.0, 0.0, 0.0]
G = [0.0, 1.0, 0.0]
B = [0.0, 0.0, 1.0]

# stack for localization
stack = deque()         
                 
                 
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
    #print("Motors stopped.")


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
    
    # get the distance values, the walls, the available turns, and the count of the available turns
    d_vals, walls, available, count = get_dvals_walls_available()
    print(f"robot localized: {localized}")
    
    # logic depending if the robot has been localized yet or not
    if localized:
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
    else:
        q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
        
        # update the robot's pose
        robot_pose = [0, 0, 0, q]
        
        # print new robot pose
        print(f"Pose: {robot_pose}")
        
    # update the direction the robot is facing and print it
    update_direction()
    print(f"Robot current grid cell: {robot_pose[2]}, Direction: {dir}")
        
    # print the visited cells
    print_visited_cells()
        
    # print the d_vals
    print(f"Distances to walls: left: {d_vals[0]}, front: {d_vals[1]}, right: {d_vals[2]}")
        
    # print the walls from the distance sensors around the robot
    print(f"Walls detected: left: {walls[0]}, front: {walls[1]}, right: {walls[2]}")
    print_walls_orientation(walls)
        
    # print the available directions that don't have walls, should be the inverse of the walls
    print(f"Directions available to go: left: {available[0]}, front: {available[1]}, right: {available[2]}")
        
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
        dir = "West"
    elif imu_val <= -45 and imu_val > -135:
        dir = "South"
    elif 45 <= imu_val <= 135:
        dir = "North"
    elif (-45 < imu_val <= 0) or (0 <= imu_val < 45):
        dir = "East"
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
            
        
def get_turns_taken(available):
    # taken are turns that have been taken already or are where available ='s 0, turns that arent possible
    # a turn ='s 1 if the robot has done a motion that would bring them to that cell from where they currrently
    # are or if that move is not possible, if the turn is possible to make and hasn't been done before, it should be left as a 0
    count = 0
    taken = [0, 0, 0]
    for i in range(len(available)):
        if available[i] == 0:
            # the move is not possible, so mark it as a turn already taken before
            taken[i] = 1
            count += 1
        elif available[i] == 1:
            # a left turn from current position that has already been taken before
            if i == 0 and grid_maze[robot_pose[2]-2][0] == 1:
                taken[0] = 1
                count += 1
            # a forward move from current position that has already been taken before
            if i == 1 and grid_maze[robot_pose[2]-5][0] == 1:
                taken[1] = 1
                count += 1
            # a right turn from current position that has already been taken before
            if i == 2 and grid_maze[robot_pose[2]][0] == 1:
                taken[2] = 1
                count += 1
    return taken, count    


def check_wall_behind(ts):
    global dir
    # sees if a wall is behind the current location, if so, make a left turn from the south position
    # now facing east
    # once the robot is facing south get the new walls
    # get the distance values, the walls, the available turns, and the count of the available turns
    d_vals, new_walls, available, count = get_dvals_walls_available()
    dir = "South"
    
    # map the walls while facing south
    add_walls_to_map(new_walls)
                
    # if turned around and a wall in the way 
    if new_walls[1] == 1:
        if dir == "South":
            if new_walls[0] == 0:  # left turn available after facing south
                turn_left(ts)
                dir = "East"
            else:
                # face north again
                turn_left(ts)
                turn_left(ts)
                dir = "North"
                stop_motors()
            return True, available
    return False, available


# map the maze, prioritizing unvisited cells
def map_maze(ts):
    while robot.step(ts) != -1:
        # once the robot is in the center of the cell and facing north again, map the cell
        # get the distance values, the walls, the available turns, and the count of the available turns
        d_vals, walls, available, count = get_dvals_walls_available()
        add_walls_to_map(walls)
        
        # after the walls have been added, see if the robot should continue mapping or not
        if check_if_robot_should_stop():        
            break 
        
        # get the turns that have already been taken and the count of turns that have already been taken
        taken, taken_count = get_turns_taken(available)
        
        #print(f"available: {available}, count: {count}")
        #print(f"taken: {taken}, taken_count: {taken_count}")
        
        # prioritize unvisited cell, do logic for turns
        if count == 0:
            # turn right twice to face south
            turn_right(ts)
            turn_right(ts)
            
            # check if after facing south if there is a wall or not
            check_wall_behind(ts)
            
        if count == 1:  # only 1 available move
            if taken_count == 3 or (available[1] == 1 and taken[1] == 1):
                # only option is forward and has been at that cell already
                # turn right twice to face south
                turn_right(ts)
                turn_right(ts)
                
                # check if after facing south if there is a wall or not
                check_wall_behind(ts)
            
            elif available[0] == 1 and taken[0] == 0:  # left turn is available
                turn_left(ts)
            
            elif available[2] == 1 and taken[2] == 0:  # right turn is available
                turn_right(ts)
            
            elif available[0] == 1 and taken[0] == 1:  # left turn is available, and has been visited before
                turn_left(ts)
            
            elif available[2] == 1 and taken[2] == 1:  # right turn is available, and has been visited before
                turn_right(ts)
        
        elif count == 2:  # 2 available moves
            if taken_count == 3:
                # face south by turning right
                turn_right(ts)
                turn_right(ts)
                
                # check if after facing south if there is a wall or not
                check_wall_behind(ts)
           
            elif taken[1] == 0:
                print("Move forward..")
           
            elif taken[0] == 0:
                turn_left(ts)
           
            elif taken[2] == 0:
                turn_right(ts)
        
        # move to the next square, by moving 10 inches in direction from from above
        move(10, ts)
        
        # visited a cell, mark it as visited
        mark_cell_visited(robot_pose[2])
        
        # make the robot face north again
        if dir != "North":
            face_north(ts)
            face_dir(ts, "North")    
    
    
# get the walls around the robot from the corresponding distance sensor measurements
def get_walls_around_robot(d_vals):
    # left, front, right
    walls = [0, 0, 0]
    for i in range(3):
        if d_vals[i] < 10.0:
            walls[i] = 1
    return walls


def print_walls_orientation(walls):
    if dir == "North":
        print(f"West: {walls[0]}, North: {walls[1]}, East: {walls[2]}, South: 0")
    
    elif dir == "South":
        print(f"West: {walls[2]}, North: 0, East: {walls[0]}, South: {walls[1]}")
    
    elif dir == "West":
        print(f"West: {walls[1]}, North: {walls[2]}, East: 0, South: {walls[0]}")
    
    elif dir == "East": 
        print(f"West: 0, North: {walls[0]}, East: {walls[1]}, South: {walls[2]}")


# changed from LAB 3: VNESW to LAB 4: VWNES
# walls are in order [left, front, right]
def add_walls_to_map(walls):
    global grid_maze
    # grid_maze is [Visited, West, North, East, South]
    #print(f"direction from awtm: {dir}")
    #print(f"walls from awtm: {walls}")
    if dir == "North":
        if walls[0] == 1:  # left wall
            grid_maze[robot_pose[2]-1][1] = 1 # left is west
            
        if walls[1] == 1:  # front wall
            grid_maze[robot_pose[2]-1][2] = 1 # front is north
            
        if walls[2] == 1:  # right wall
            grid_maze[robot_pose[2]-1][3] = 1 # right is east
        
    if dir == "South":
        if walls[0] == 1:  # left wall
            grid_maze[robot_pose[2]-1][3] = 1 # left is east
            
        if walls[1] == 1:  # front wall
            grid_maze[robot_pose[2]-1][4] = 1 # front is south
            
        if walls[2] == 1:  # right wall
            grid_maze[robot_pose[2]-1][1] = 1 # right is west 
    
        
    # add the added walls to the accompanying cells nearby
    for cell in range(num_cells):
        # grid_maze is [Visited, West, North, East, South]
        # if the current cell and the cell to the right 
        # both share an inner wall, if one cell is set to 
        # 1, set both of them to 1 for their E/W wall
        if cell < (num_cells - 1):  # cell < 15
            if grid_maze[cell][3] != grid_maze[cell+1][1]:
                grid_maze[cell][3] = 1
                grid_maze[cell+1][1] = 1
        else:  # cell 15
            if grid_maze[cell][1] != grid_maze[cell-1][3]:
                grid_maze[cell][1] = 1
                grid_maze[cell-1][3] = 1
        # if the current cell and the cell beneath it 
        # both share an inner wall, if one cell is set to 
        # 1, set both of them to 1 for their N/S wall
        if cell < (num_cells - 4):  # cell < 12
            if grid_maze[cell][4] != grid_maze[cell+4][2]:
                grid_maze[cell][4] = 1
                grid_maze[cell+4][2] = 1
        else: # cells 13 to 15
            if grid_maze[cell][2] != grid_maze[cell-4][4]:
                grid_maze[cell][2] = 1
                grid_maze[cell-4][4] = 1


def print_visited_cells():
    pr_vc = ""
    for i in range(len(visited_cells)):
        if i % 4 == 0 and i != 0:
            pr_vc += "\n"
        pr_vc += visited_cells[i]
    print("Visited cells:")
    print(pr_vc)
        

# changed from LAB 3: VNESW to LAB 4: VWNES
def print_maze(maze):
    print("________________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x+1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x+2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x+3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
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


def face_dir(ts, direction="North"):
    # get the destination angle for the direction
    if direction == "North":
        dir_angle = 90.0
    elif direction == "West":
        dir_angle = 180.0
    elif direction == "East":
        dir_angle = 0.0
    elif direction == "South":
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
        #print(f"diff: {diff}")
        #print(f"abs_diff: {abs_diff}")
        print(f"Direction: {get_direction(q)}")
        print(f"Rotating until facing: {direction}..")
        
        # slow the speed down, or flip direction based on abs_diff 
        if diff < 0 or (180 <= abs_diff <= 270):
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


# get the cylinder from the color detected
def get_cyl(color):
    if color is None:
        return None
    elif color == Y:
        return y_cyl
    elif color == R:
        return r_cyl
    elif color == G:
        return g_cyl     
    elif color == B:
        return b_cyl


def get_dvals_walls_available():
    # once the robot is in the center of the cell and facing north again, map the cell
    d_vals = get_d_sensors_vals()
    walls = get_walls_around_robot(d_vals)           
    # get available turns for the walls that are shown
    available, count = get_available_turns(walls)
    return d_vals, walls, available, count
    

def look_for_cyl_in_cell(ts, range):
    color = None
    # rotate 1 rotation to see if a cylinder is right next to the robot
    seconds = 6.5
    # get rotation speed for full rotation across duration of seconds
    left, right = get_rot_speed_rad(359.5, seconds)
    end_time = seconds + robot.getTime()
    while robot.step(ts) != -1:
        # print the time
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        print("Looking for cylinder in current cell..")
        if robot.getTime() < end_time:
            leftMotor.setVelocity(left)
            rightMotor.setVelocity(right)
            # get the color of the closest cylinder from the camera
            objs = camera.getRecognitionObjects()
            
            if len(objs) > 0:
                # get the first object
                obj = objs[0]
                # get the objects relative angle and relative distance 
                # from the robot to the recognized color/cylinder
                pos = obj.get_position()
                rel = math.fabs(m_to_i(pos[2]))
                print(f"Distance to cylinder: {rel}")
                if rel < range:
                    # get the object's recognized color
                    color = obj.get_colors()
                    break
        else:
            stop_motors()
            break
    if color is None:
        # make the robot face absolutely north
        face_dir(timestep, "North")
    return color


def localize_robot(ts):
    # range to be within for looking for the cylinder
    range = 5.5
    starting_point = [0,0]
    # get the cylinder returned from the color from the rotation search
    cyl = get_cyl(look_for_cyl_in_cell(ts, range))
    
    if cyl is None:
        while robot.step(ts) != -1:
            # get the color of the closest cylinder from the camera
            objs = camera.getRecognitionObjects()
            
            if len(objs) > 0:
                # get the first object
                obj = objs[0]
                
                # get the objects relative angle and relative distance 
                # from the robot to the recognized color/cylinder
                pos = obj.get_position()
                rel = math.fabs(m_to_i(pos[2]))
                
                # get the object's recognized color
                color = obj.get_colors()
                
                # from the cylinder color, get which cylinder array is associated with it
                cyl = get_cyl(color)
                    
                #print(f"Color of cylinder: {color}")
                #print(f"Relative distance to cylinder: {rel}")
                
                if rel > range:    
                    navigate_to_cylinder(ts)
                else:
                    break
            else:  # no cylinder has been found at all
                print("Can't see a cylinder..")
                navigate_to_cylinder(ts) 
    
    # clear the stack
    stack.clear()
        
    # get the starting point coordinates to map the inner
    # walls now that the robot has localized next to a cylinder
    if cyl == y_cyl:
        starting_point = [-15.0, 15.0]
    if cyl == r_cyl:
        starting_point = [15.0, 15.0]
    if cyl == g_cyl:
        starting_point = [-15.0, -15.0]
    if cyl == b_cyl:
        starting_point = [15.0, -15.0]
    return cyl, starting_point


def navigate_to_cylinder(ts):
    global stack
    
    # make the robot face north first
    if dir != "North":
        face_north(ts)
    
    #print(f"stack size: {len(stack)}")
    #print(f"stack: {stack}")
    
    # get the distance values, the walls, the available turns, and the count of the available turns
    d_vals, walls, available, count = get_dvals_walls_available()
    #print(f"walls: {walls}")
    #print(f"available: {available}, count: {count}")
    
    if len(stack) == 0:
        if count == 0:
            # only option is forward and has been at that cell already
            # turn right twice to face south
            turn_right(ts)
            turn_right(ts)
                    
            # check if after facing south if there is a wall or not
            behind, new_available = check_wall_behind(ts)
            if behind:
                if dir == "East":
                    # looking south, this was a left turn
                    stack.append("left")
                elif dir == "West":
                    # looking south, this was a right turn
                    stack.append("right")
            else:
                stack.append("reverse")
        elif available[1] == 1:
            stack.append("forward")
            print("move forward..")
        elif available[0] == 1:
            turn_left(ts)
            stack.append("left")
        elif available[2] == 1:
            stack.append("right")
            turn_right(ts)
    else:  # previous moves have been made
        #print(f"last move: {stack[len(stack)-1]}")
        if available[1] == 1 and stack[len(stack)-1] != "reverse":
            stack.append("forward")
            print("move forward..")
        elif available[0] == 1 and stack[len(stack)-1] != "right":
            turn_left(ts)
            stack.append("left")
        elif available[2] == 1 and stack[len(stack)-1] != "left":
            stack.append("right")
            turn_right(ts)
        else:
            count = 0
            #print(f"new count: {count}")
        if count == 0:
            # only option is forward and has been at that cell already
            # turn right twice to face south
            turn_right(ts)
            turn_right(ts)
                    
           # check if after facing south if there is a wall or not
            behind, new_available = check_wall_behind(ts)
            if behind:
                if dir == "East":
                    # looking south, this was a left turn
                    stack.append("left")
                elif dir == "West":
                    # looking south, this was a right turn
                    stack.append("right")
            else:  # no wall behind, drive, south
                stack.append("reverse")
    move(10.0, ts)
            

def main():
    global localized
    global new_pos
    global robot_pose
    # map the maze, prioritizing unvisited cells
    while robot.step(timestep) != -1:
        # localize the robot next to a cylinder
        cylinder, pos = localize_robot(timestep)
        
        # now the robot has been localized, set this to True now
        localized = True
        
        if localized: 
            # make the robot face absolutely north
            face_dir(timestep, "North")
            
            # update the robot_pose
            robot_pose = [
            pos[0], pos[1], 
            get_current_grid_cell(pos[0], pos[1]), 
            ((imu.getRollPitchYaw()[2] * 180) / 3.14159)
            ]
            # update the new_pos
            new_pos = [pos[0], pos[1]]
            
            # now that the robot knows what position it is at, 
            # mark it on the map and start mapping the maze
            mark_cell_visited(robot_pose[2])
            map_maze(timestep)
            
            # check after every move to see if the robot should stop or not
            if check_if_robot_should_stop():
                print("Final maze with walls mapped:")
                print_maze(grid_maze)
                break                  


if __name__ == "__main__":
    main()
