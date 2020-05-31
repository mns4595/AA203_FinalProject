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
RRT_star_waypoints = waypoints;

%% Test - Visualize RRT* Solution & Obstacles with Refined Waypoints
if test
    %     result.plot() %This takes ~5-10sec to render. Displays total cost too
    PlotTrajectory(waypoints, obstacles, polyOrder)
end

%% Initial Polynomial Fit
polyOrder = 9;
iteration = 1;
new_waypoint_exists = true;
final_times=[];

while iteration <= 10 && new_waypoint_exists%% Need to add error constraint
    iteration  = iteration + 1;
    new_waypoint_exists = false; % If this is not changed, loop will end with this pass
    
    %% Time Optimization
    % Sets large time estimates for initial
    initial_times = ones(1,length(waypoints)) * 4;
    % Minimize segment times to minimize snap
    
    %%%%%%%%%%%%%%%% TO DO: Finish Optimize_Time_Ratio  %%%%%%%%%%%%%%%%%%
    % Outputs the Trajectories and time ratios for later use
    [new_times, xTraj, yTraj] = Optimize_Time_Ratio(waypoints, initial_times, polyOrder);
    if test
        PlotTrajectory(waypoints, obstacles, polyOrder, xTraj, yTraj);
    end
    
    %% Collision Check
    % Use collision_free to ensure no trajectory points overlap the
    % obstacle regions
    for segment_number = 1:length(xTraj)
        if ~new_waypoint_exists % stops this loop if a new waypoint already identified
            for segment_step = 1:(length(xTraj{segment_number}) - 1)
                current_point = [xTraj{segment_number}(segment_step), yTraj{segment_number}(segment_step)];
                next_point = [xTraj{segment_number}(segment_step+1), yTraj{segment_number}(segment_step+1)];
                
                % Checking to see if the straight line between the current and
                % next time step overlaps an obstacle boundary
                if ~collision_free(obstacles, current_point, next_point)
                    % add waypoint between the two endpoints of the current
                    % segment
                    extra_waypoint = (waypoints(segment_number,:) + waypoints(segment_number + 1,:))/2;
                    % Updates waypoints to add an additional one where the
                    % collision was detected
                    waypoints = [waypoints(1:segment_number, : ); extra_waypoint; waypoints(segment_number+1:end, : )];
                    new_waypoint_exists = true;
                    break
                end
            end
        end
    end
    
    %% Update Times to Meet Actuator Constraints
    % With times optimized and no collisions, times are increased to the
    % step greater than the limit where a constraint is hit or decreased
    % until a constraint is hit and reverted back by one step.
    
    %%%%%%%%%%%%%%%% TO DO  %%%%%%%%%%%%%%%%%%%%%%
    final_times = Constrain_Times_For_Actuator(new_times);
    
end

% Output the final results
[xCoeff,yCoeff,xTraj,yTraj,cost] = TrajOpt(waypoints,final_times,polyOrder);
PlotTrajectory(waypoints, obstacles, polyOrder, xTraj, yTraj);
