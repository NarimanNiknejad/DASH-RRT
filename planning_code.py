from turtle import width
import pygame 

import logging
import csv
import cvxpy as cp
import numpy as np
import scipy.linalg as linalg
# import control
import random
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import math
import scipy.stats as stats

from planning_base import RRTGraph
from planning_base import RRTMap
import time
from scipy.signal import cont2discrete
from control import dlqr


Real_MapDimension = (1.05,1.05)
dimensions = (800,800)

min_x = -Real_MapDimension[0]
max_x = Real_MapDimension[0]
min_y = -Real_MapDimension[1]
max_y = Real_MapDimension[1] 


Maph = dimensions[0]
Mapw = dimensions[1]

start = (-0.7,-0.7)
goal = (0.7,0.7)
obsdim = 0.35
obsnum = 1

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BOUNDARY_COLOR = (150, 190, 30)  



def map_to_screen(x_meters, y_meters):

    screen_width = Mapw
    screen_height = Maph

    # Calculate the scale factors for conversion
    x_scale = screen_width / (max_x - min_x)
    y_scale = screen_height / (max_y - min_y)
    
    x_pixels = (x_meters - min_x) * x_scale
    y_pixels = (-y_meters + max_y) * y_scale

    return int(x_pixels), int(y_pixels)

# Draw axes with numbers
def draw_axes(screen,HEIGHT=Maph,WIDTH=Mapw):

    # Constants for scaling
    x_min, x_max = -1.05, 1.05
    y_min, y_max = -1.05, 1.05

    # Scaling factors (pixels per unit)
    x_scale = WIDTH / (x_max - x_min)
    y_scale = HEIGHT / (y_max - y_min)

    # Zero positions for x and y axes in terms of pixels
    x_origin = 70  # Left edge for the y-axis
    y_origin = HEIGHT - 50   # Bottom edge for the x-axis

    # Draw x-axis (along the bottom)
    pygame.draw.line(screen, (0, 0, 0), (0, y_origin), (WIDTH, y_origin), 2)
    for i in range(-5, 6, 1):  # i goes from -5 to 5, corresponding to -1.05 to 1.05
        x = int(x_scale * (i * 0.21) + WIDTH // 2)
        pygame.draw.line(screen, (0, 0, 0), (x, y_origin - 5), (x, y_origin + 5), 2)
        font = pygame.font.Font(None, 35)
        text = font.render(f"{i * 0.21:.2f}", True, (0, 0, 0))
        text_rect = text.get_rect(center=(x, y_origin + 20))
        screen.blit(text, text_rect)

    # Draw y-axis (along the left)
    pygame.draw.line(screen, (0, 0, 0), (x_origin, 0), (x_origin, HEIGHT), 2)
    for i in range(-5, 6, 1):
        y = int(HEIGHT - y_scale * ((i) * 0.21)- HEIGHT // 2)  # Multiply by 0.21 to match the tick interval
        pygame.draw.line(screen, (0, 0, 0), (x_origin - 5, y), (x_origin + 5, y), 2)
        font = pygame.font.Font(None, 35)
        text = font.render(f"{i * 0.21:.2f}", True, (0, 0, 0))
        text_rect = text.get_rect(center=(x_origin - 40, y))
        screen.blit(text, text_rect)

    # Add x and y axis labels at the centers
    font = pygame.font.Font(None, 40)
    x_label = font.render("x (m)", True, (0, 0, 0))
    x_label_rect = x_label.get_rect(center=(WIDTH // 2, HEIGHT - 70))
    screen.blit(x_label, x_label_rect)
    
    y_label = font.render("y (m)", True, (0, 0, 0))
    y_label_rect = y_label.get_rect(center=(110, HEIGHT // 2))
    screen.blit(y_label, y_label_rect)





def inside_ellipsoidal(P,x_c,y_c,x,y):
    P = np.array(P)
    # Calculate the ellipsoid equation for each point
    z = (x-x_c)**2 * P[0,0] + (y-y_c)**2 * P[1,1] + 2 * (x-x_c) * (y-y_c) * P[0,1]
    if(z>1.0):
        return False
    else:
        return True

def cost(state_,input_,Q,R):

    return np.dot(np.dot(state_.T, Q), state_) + np.dot(np.dot(input_.T, R), input_)


def drawEllip(map,color,x_c,y_c,Pss):

    # print(x_c,y_c,Pss)

    # Define the range for x and y
    x_range = np.linspace(min_x, max_x, Mapw)
    y_range = np.linspace(min_y, max_y, Maph)

    # Create a grid of points
    x, y = np.meshgrid(x_range, y_range)

    P = np.array(Pss)
    # Calculate the ellipsoid equation for each point
    z = (x-x_c)**2 * P[0,0] + (y-y_c)**2 * P[1,1] + 2 * (x-x_c) * (y-y_c) * P[0,1]

    # Draw the contour lines of the ellipsoid
    for level in [1.0]:
        for i in range(Mapw):
            for j in range(Maph):
                if abs(z[j, i] - level) < 0.01:
                    x_pixel, y_pixel = map_to_screen(x[j, i], y[j, i])
                    pygame.draw.circle(map, color, (x_pixel, y_pixel), 1)

def computeUbar(x,y,A,B):
    n, m = B.shape
    c = np.array([[1,0,0],[0,1,0]])
    M1 = np.hstack([A-np.eye(n),B])
    M2 = np.hstack([c,np.zeros((2,4))])

    M = np.vstack([M1,M2])
    r = np.array([[0],[0],[0],[x],[y]])
    
    mid = np.linalg.inv(M)@r
    
    print(mid)

    uBar = np.array([mid[-2],mid[-1]])

    return uBar

def main():
    # Define Gaussian disturbance characteristics
    mu = 0       # Mean of the Gaussian disturbance (can be 0 for no bias)
    sigma_x1 = 0.1  # Standard deviation for the disturbance on x1
    sigma_x2 = 0.0  # Standard deviation for the disturbance on x2

    # Define the matrices of the system
    t_step = 0.01
    L_ab = 0.22

    A = np.eye(3)
    B = np.array([[t_step*L_ab,t_step*L_ab,t_step*L_ab,t_step*L_ab],
                [t_step*L_ab,-t_step*L_ab,t_step*L_ab,-t_step*L_ab],
                [t_step,-t_step,-t_step,t_step]])

    W = 0

    # State and input dimensions
    n, m = B.shape

    # Contractivity factor
    lamda = 0.95

    # H_inf parameters
    C = 1*np.array([[1,0,0]])
    D = 0*np.array([[1,1,1,1]])

    goal_radius= 0.2
    tol = 0.1
    iteration = 0
    gamma = 1.5
    eta = gamma**(-2)
    pygame.init()
    map = RRTMap(start,goal, dimensions,Real_MapDimension,obsdim, obsnum,goal_radius=goal_radius)
    graph = RRTGraph(start,goal,dimensions,Real_MapDimension,obsdim, obsnum, A, B, W, C, D, lamda,eta,goal_radius=goal_radius)

    obstacles = graph.makeObs_fromMeters_at_center()
    map.drawMap(obstacles)
    draw_axes(map.map)
    pygame.draw.rect(map.map, BOUNDARY_COLOR, (0, 0, Maph, Mapw), 8)

    clock = pygame.time.Clock()


    while(not graph.path_to_goal()):
        print(f"Iteration: {iteration}", end='\r', flush=True)
        if iteration%10==0:
            X, Y, Parent, P, borders_, SUC = graph.expand_from_goal()     
        else:
            X, Y, Parent, P, borders_, SUC = graph.expand() 

        if SUC == 1:
            # border_ = borders_[-1]
            x_node, y_node = map_to_screen(X[-1],Y[-1])
            x_parent, y_parent = map_to_screen(X[Parent[-1]],Y[Parent[-1]])
            # pygame.draw.circle(map.map,map.Red,(x_node,y_node),map.nodeRad,map.nodeThickness)
            # pygame.draw.line(map.map,map.Blue,(x_node,y_node),(x_parent, y_parent),map.edgeThickness)
            # print(P[-1])


            # drawEllip(map.map,map.elColor,X[-1],Y[-1], P[-1])


            # print(border_)
            # print(borders_)

            # x_b, y_b = map_to_screen(border_[1],border_[2])
            # dim_x_b = (border_[0] - border_[1])*Mapw/(2*max_x)
            # dim_y_b = (border_[2] - border_[3])*Maph/(2*max_y)
            # rectang_b=pygame.Rect((x_b, y_b),(dim_x_b ,dim_y_b))
            # pygame.draw.rect(map.map,map.Blue,rectang_b,3)

        if iteration%1 ==0:
            pygame.display.update()
        iteration = iteration + 1

    map.drawPath(graph.getPathCoords_in_pixels(),graph.getEllipsoid_along_path())


    gains = graph.getGains_along_path()
    centers = graph.getPathCoords()
    ellipsoids = graph.getEllipsoid_along_path()
    # print(centers[1])
    # print(gains)

    
    loc_now = [centers[0][0],centers[0][1],0]
    loc_now = np.array(loc_now).reshape((n, 1))
    loc_final = [centers[-1][0],centers[-1][1],0]
    loc_final = np.array(loc_final).reshape((n, 1))
    final_loc = np.array([[centers[-1][0]],[centers[-1][1]]])
    
    h = 1
    # exit()
    
    while (h<len(centers)):

        x_old = loc_now[0][0]
        y_old = loc_now[1][0]
        print([x_old,y_old])

        for e in range(h,len(ellipsoids)):
            if(inside_ellipsoidal(ellipsoids[e],centers[e][0],centers[e][1],x_old,y_old)):
                h = e

        loc_next = [centers[h][0],centers[h][1],0]
        loc_next = np.array(loc_next).reshape((n, 1))
    

        U = gains[h]@(loc_now-loc_next) 
        # Create disturbance vector (adding noise to first two states only)

        disturbance = np.array([
            np.random.normal(mu, sigma_x1),  # Disturbance on x1
            np.random.normal(mu, sigma_x2),  # Disturbance on x2
            0                                # No disturbance on x3
        ])

        loc_now = A@(loc_now) + B@U + disturbance

        x_new = loc_now[0][0]
        y_new = loc_now[1][0]

        x_new_pixel, y_new_pixel=map_to_screen(x_new,y_new)
        x_old_pixel, y_old_pixel=map_to_screen(x_old,y_old)
        # pygame.draw.circle(map.map,map.colors[h+1],(x_new_pixel,y_new_pixel),map.nodeRad+5)
        now_loc = np.array([x_new,y_new])
        now_loc = np.array(now_loc).reshape((2, 1))
        pygame.draw.line(map.map,map.colors[h],(x_old_pixel,y_old_pixel),(x_new_pixel, y_new_pixel),map.edgeThickness+3)
        pygame.display.update()   

        # print(now_loc)
        # print('fin:')
        # print(final_loc)
        # print(final_loc)

        if(np.linalg.norm(now_loc-final_loc) <=tol):
            break

    print("\nDASH done")
    pygame.image.save(map.map, "DASH-RRT.png")
    # update display

    running = True
    while running == True:
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                running = False
    
    pygame.quit()

if __name__=='__main__':
    main()