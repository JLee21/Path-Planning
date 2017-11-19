# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
#
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------
from termcolor import cprint

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]]
h = [[9, 8, 7, 6, 5, 4],
     [8, 7, 6, 5, 4, 3],
     [7, 6, 5, 4, 3, 2],
     [6, 5, 4, 3, 2, 1],
     [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost,heuristic):

    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0

    open = [[h[x][y], g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail", expand
        else:
            open = [x for x in open if len(x) == 4]
            [cprint('after open append {}'.format(x), 'yellow') for x in open]
            fs = [x[0] for x in open]
            gs = [x[1] for x in open]
            fs.sort(); gs.reverse()
            gs.sort(); gs.reverse()
            print('fs sorted and reverse', fs)
            print('gs sorted and reverse', gs)
            next_f = fs[-1]
            next_g = gs[-1]
            if len(gs) > 1:
                print(gs)
                if gs[-1] == gs[-2]: next_f = 0

            cprint('{} < {}'.format(next_f, next_g), 'red')
            if next_f < next_g:
                open.sort()
                open.reverse()
                next = open.pop()
                cprint('f is lower {}'.format(next), 'cyan')
                g = next[1]
                x = next[-2]
                y = next[-1]
            else:
                open = [x[1:] for x in open]
                open.sort()
                open.reverse()
                next = open.pop()
                cprint('g is lower {}'.format(next), 'red')
                g = next[0]
                x = next[-2]
                y = next[-1]

            cprint('count: {:2d} f: {:2d} g: {:2d} next {}'
                .format(count, next_f, next_g, next), 'blue', 'on_grey')

            expand[x][y] = count
            count += 1

            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            f = h[x2][y2] + g
                            cprint('before open.append {} {} {} {}'
                                .format(f, g2, x2, y2), 'grey', 'on_white')
                            open.append([f, g2, x2, y2])
                            closed[x2][y2] = 1

    return 'Success!!\n', expand

err, expand = search(grid,init,goal,cost, h)

print(err)
for i in expand:
    print(i)
