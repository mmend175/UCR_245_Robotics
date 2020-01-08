

def user_input():
    # Initialize Map Size
    map_size = [0, 0, 0, 20, 20, 20]
    '''map_size[0] = int(input("Choose x origin for your map: "))
    map_size[1] = int(input("Choose y origin for your map: "))
    map_size[2] = int(input("Choose z origin for your map: "))
    map_size[3] = int(input("Choose width of your map: "))
    map_size[4] = int(input("Choose length of your map: "))
    map_size[5] = int(input("Choose height of your map: ")) '''

    # Initialize Map Obstacles
    num_obs = int(input("Choose how many obstacles there will be: "))
    obstacles = [[0 for j in range(6)] for k in range(num_obs)]

    for i in range(num_obs):
        obs_array = [0, 0, 0, 0, 0, 0]
        obs_array[0] = int(input("Choose x origin for obstacle " + str(i + 1) + ":"))
        obs_array[1] = int(input("Choose y origin for obstacle " + str(i + 1) + ":"))
        obs_array[2] = int(input("Choose z origin for obstacle " + str(i + 1) + ":"))
        obs_array[3] = int(input("Choose width for obstacle " + str(i + 1) + ":"))
        obs_array[4] = int(input("Choose length origin for obstacle " + str(i + 1) + ":"))
        obs_array[5] = int(input("Choose height origin for obstacle " + str(i + 1) + ":"))
        obstacles[i] = obs_array

    # Map limits
    max_x = map_size[3] - map_size[0]
    max_y = map_size[4] - map_size[1]
    max_z = map_size[5] - map_size[2]

    # Initialize q0
    q0 = [0, 0, 0]
    q0_x = int(input("Choose initial x position for your quadrotor: "))
    while (q0_x > max_x) or (q0_x < map_size[0]):
        print("INVALID x pos,  try again...")
        q0_x = int(input("Choose initial x position for your quadrotor: "))
    q0[0] = q0_x
    q0_y = int(input("Choose initial y position for your quadrotor: "))
    while (q0_y > max_y) or (q0_y < map_size[1]):
        print("INVALID y pos,  try again...")
        q0_y = int(input("Choose initial y position for your quadrotor: "))
    q0[1] = q0_y
    q0_z = int(input("Choose initial z position for your quadrotor: "))
    while (q0_z > max_z) or (q0_z < map_size[2]):
        print("INVALID z pos,  try again...")
        q0_z = int(input("Choose initial z position for your quadrotor: "))
    q0[2] = q0_z

    # Initialize qh
    qh = [max_x, max_y, max_z]
    qh_x = int(input("Choose final x position for your quadrotor: "))
    while (qh_x > max_x) or (qh_x < map_size[0]):
        print("INVALID x pos,  try again...")
        qh_x = int(input("Choose final x position for your quadrotor: "))
    qh[0] = qh_x
    qh_y = int(input("Choose final y position for your quadrotor: "))
    while (qh_y > max_y) or (qh_y < map_size[1]):
        print("INVALID y pos,  try again...")
        qh_y = int(input("Choose final y position for your quadrotor: "))
    qh[1] = qh_y
    qh_z = int(input("Choose final z position for your quadrotor: "))
    while (qh_z > max_z) or (qh_z < map_size[2]):
        print("INVALID z pos,  try again...")
        qh_z = int(input("Choose final z position for your quadrotor: "))
    qh[2] = qh_z

    return map_size, obstacles, q0, qh
