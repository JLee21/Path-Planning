# ----------
# User Instructions:
#
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1
global cnt
cnt = 0

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

archive = []


# state = [<g_value>, <x>, <y>]
def move(state_g, cnt):

    cnt += 1
    path = []
    cprint('*'*64, 'blue', 'on_white')
    cprint('[state_g]: {}'.format(state_g), 'cyan')
    if cnt == 1:
        archive.append([state_g[1], state_g[2]])

    for x in delta:
        foo = [0,0,0] # to hold each new moved state and its cost
        state = state_g[1:]
        new_state = [0,0]
        new_state[0] = x[0] + state[0]
        new_state[1] = x[1] + state[1]
        # cprint('[new_state]: {}'.format(new_state), 'red', 'on_grey')

        # check if we've gone here before
        for x in archive:
            # cprint('[archive]: {}'.format(x), 'white', 'on_green')
            if (x[0]==new_state[0] and x[1]==new_state[1]):
                # print('[INSIDE ARCHIVE]')
                # print('x[0] {} x[1] {}'.format(x[0], x[1]))
                # print('new_state[0] {} new_state[1] {}'.format(new_state[0], new_state[1]))
                continue
        # if you made it this far, then archive
        archive.append(new_state)

        # check if we're at the goal
        if (new_state[0] == goal[0] and new_state[1] == goal[1]):
            print('Finished!')
            exit
        # check if we are out of bounds
        if (0 > new_state[0] or new_state[0] > len(grid[0])):
            cprint('out of bounds row: 0 > {} > {}'.format(new_state[0], len(grid[0])), 'red')
            continue
        if (0 > new_state[1] or new_state[1] > len(grid)):
            cprint('out of bounds col: 0 > {} > {}'.format(new_state[1], len(grid)), 'red')
            continue
        # check if we hit the wall
        x = new_state[1]; y = new_state[0]
        if (grid[x][y]):
            print('hit a wall: grid val: {} new_state: {}'.format(grid[x][y], new_state), 'red')
            continue

        # add the cost of approved new_state
        foo[0] = cost + state_g[0]
        foo[1:] = new_state
        cprint('[*]: {}'.format(foo), 'green', 'on_grey')

        # save the approved states (here, up to four possible)
        path.append(foo)

    for x in archive:
        cprint('[archive]: {}'.format(x), 'white', 'on_blue')

    # choose the lowest cost new_state AKA `x`
    for i, x in enumerate(path):
        g = x[0]
        cprint('[possible path]: {}'.format(x), 'grey', 'on_white')
        if i == 0: lowest = g; continue
        if g <= lowest:
            print('[lowest g value]', lowest)
            lowest = i


    if (cnt > 5): raise Exception('reached counter')
    # send the new_state with the lowest g value back to move()
    cprint('[sending]: {}'.format(path[lowest]), 'yellow')
    move(path[lowest], cnt)

def search(grid, init, goal, cost):
    '''
    '''
    # initial open list
    cprint('[init]: {}'.format(init), 'magenta')
    cprint('[goal]: {}'.format(goal), 'green')
    state = [0,0,0]
    state[1:] = init
    move(state, cnt)

if __name__ == "__main__":
    from termcolor import cprint
    search(grid, init, goal, cost)
