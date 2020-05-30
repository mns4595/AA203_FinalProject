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
    
    figure('Renderer', 'painters', 'Position', [1920/2 50 1920/3 900])
    for i = 1: length(obstacles)
        fill(obstacles{i}(:,1), obstacles{i}(:,2), 'k')
        hold on
    end
    goal = fill([8 8 10 10 8], [16 18 18 16 16], 'g');
    set(goal, 'facealpha', 0.5)
    set(goal, 'edgealpha', 0)

    plot(waypoints(:,1), waypoints(:,2))
    scatter(waypoints(:,1), waypoints(:,2), 'linewidth', 1)
    xlim([0 10])
    ylim([0 18])
    
end

%% Polynomial Fit







