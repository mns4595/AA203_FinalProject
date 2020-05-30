%% AA 203 - Optimal and Learning Based Controls: Final Project
%
%-------------------------------------------------------------------------%
% This MATLAB Script intends to reproduce the results obtained from the
% "Polynomial Trajectory PLanning for Aggressive Quadrotor Flight in Dense
% Indoor Environments" research paper by Charles Richter, Adam Bry, and 
% Nicholas Roy located at:
% https://dspace.mit.edu/bitstream/handle/1721.1/106840/Roy_Polynomial%20trajectory.pdf?sequence=1&isAllowed=y
%-------------------------------------------------------------------------%
%                                                                         %
%  Written by: Tom McClure, Marco Nunez                                   %
%              Department of Aeronautics & Astronautics                   %
%              Stanford University                                        %
%                                                                         %
%              Will Harvey                                                %
%              Department of Mechanical Engineering                       %
%              Stanford University                                        %
%                                                                         %
%-------------------------------------------------------------------------%
% rrt-toolbox was used in order to solve for RRT*. Some functions and     %
% classes were modified to improve collision checking and solution finding%
%-------------------------------------------------------------------------%


%% Setup
clear
close all
clc

% Testing Mode
test = true;
% test = false;

% RRT* solver setup
addpath('rrt_toolbox-master');
load('obstacles');

grid_size = 10;

map = struct('name', 'poly_world2.mat', 'start_point', [0.5 0.5], 'goal_point', [9 16]);
max_iter =  2600;
is_benchmark = false;
rand_seed = 200;
variant = 'FNSimple2D';


%% Matlab RRT* Waypoints

polyOrder = 1;
result = rrt_star(map, max_iter, is_benchmark, rand_seed, variant);
path_idx = result.getResultantPath();
waypoints = zeros(length(path_idx), 2);
waypoints(:,1) = result.tree(1,path_idx);
waypoints(:,2) = result.tree(2,path_idx);

waypoints = flip(waypoints);

% Refine waypoints
success = false;
while ~success
    success = true;
    n = length(waypoints(:,1));
    for i = 2:n-1
        parent_idx = i-1;
        child_idx = i+1;
        
        if collision_free(obstacles, waypoints(parent_idx,:), waypoints(child_idx,:))
            waypoints(i,:) = [];
            success = false;
            break
        end
    end
end

%% Test - Visualize RRT* Solution & Obstacles with Refined Waypoints
if test
%     result.plot() %This takes ~5-10sec to render. Displays total cost too
    PlotTrajectory(waypoints, obstacles, polyOrder)    
end

%% Initial Polynomial Fit

% initial_times=[2,.4,.5,3];
% Sets large time estimates for initial 
initial_times = ones(1,length(waypoints)) * 4;
total_time = sum(initial_times);
polyOrder = 9;
[xCoeff,yCoeff,xTraj,yTraj,cost] = TrajOpt(waypoints,initial_times,polyOrder);
PlotTrajectory(waypoints, obstacles, polyOrder, xTraj, yTraj)

iteration = 1;
while iteration <= 100 %% Need to add error constraint
    iteration  = iteration + 1;
    %% Time Optimization
    % Minimize segment times to minimize snap
    %%%%%%%%%%%%%%%% TO DO  %%%%%%%%%%%%%%%%%%%%%%
    
    
    %% Collision Check
    % Use collision_free to ensure no trajectory points overlap the
    % obstacle regions
    %%%%%%%%%%%%%%%% TO DO  %%%%%%%%%%%%%%%%%%%%%%
    
    %% Update Times to Meet Actuator Constraints
    % With times optimized and no collisions, times are increased to the
    % step greater than the limit where a constraint is hit or decreased
    % until a constraint is hit and reverted back by one step.
    %%%%%%%%%%%%%%%% TO DO  %%%%%%%%%%%%%%%%%%%%%%
    
end


