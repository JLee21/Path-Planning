# ----------
# User Instructions:
#
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal.
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------
#
from termcolor import cprint

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid,goal,cost):

    value = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    value[goal[0]][goal[1]] = 0
    pol = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[goal[0]][goal[1]] = 1

    x = goal[0]
    y = goal[1]
    cnt = 0

    def move(states):
        if len(states) == 0:
            print('Done with move()')
            return
        # cache `states`
        states_ = states; states = []
        for state in states_:
            x = state[0]
            y = state[1]
            g = state[2]
            cprint('for state in states_ {} {} {}'.format(x, y, g), 'white', 'on_grey')
            for i in range(len(delta)):
                # perform move
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]
                # validate move
                if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                    if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                        # update value-grid with new cost
                        new_cost = g + cost
                        value[x2][y2] = new_cost
                        # cache move to lists `states` and `closed`
                        states.append([x2, y2, new_cost])
                        closed[x2][y2] = 1
                        pol[x2][y2] = delta_name[i]
        move(states)

    # begin recusrive search
    move([[x,y,0]])

    # make sure your function returns a grid of values
    return value, pol

value, pol = compute_value(grid, goal, cost)

for v in value:
    print(v)

for p in pol:
    print(p)
