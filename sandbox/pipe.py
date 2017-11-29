import subprocess
import os; os.system('clear')
from termcolor import cprint
import io
import time
import sys
import argparse
import pygame

parser = argparse.ArgumentParser()
parser.add_argument("-m", action='store_true')
args = parser.parse_args()

# GUI
pygame.init()
size = width, height = 400, 400
speed = [2, 2]
color = 20, 20, 20
screen = pygame.display.set_mode(size)

# mouse click
if args.m:
    import pyautogui as gui
    gui.click(300, 400)
    gui.click(300, 400)
    gui.click(300, 400)

# Make the C++ program
cmd = ['cd ../build && make']
proc_make = subprocess.Popen(cmd,
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE, shell=True)
print('make pid ', proc_make.pid)
passing = False
while not passing:
    try:
        line = proc_make.stdout.readline().rstrip()
        if line: print(line)
        if b'100%' in line:
            proc_make.kill()
            passing = True
    except KeyboardInterrupt:
        proc_make.kill()
        exit()

path = os.path.abspath("../build/run"); cprint(path, 'blue')
proc = subprocess.Popen([path],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE)
cprint('PID: {}'.format(proc.pid), 'blue')

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    try:
        line = proc.stdout.readline().rstrip()
        print(str(line))
    except KeyboardInterrupt:
        cprint('\nExiting...', 'red')
        proc.kill()
        sys.exit()

    screen.fill(color)
    # screen.blit(ball, ballrect)
    pygame.display.flip()
