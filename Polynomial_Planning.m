%% AA 203 Final Project
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
% *The Python Code was extended from materials produced in AA 274A Robot  %
%  Autonomy I Homework 2 from the Autumn quarter of 2019                  %
%-------------------------------------------------------------------------%

%% Setup
clear
close all
clc

% RRT* solver setup
addpath('rrt_toolbox-master');

grid_size = 10;

map = struct('name', 'poly_world.mat', 'start_point', [0.5 0.5], 'goal_point', [9 16]);
max_iter =  2000;
is_benchmark = false;
rand_seed = 40;
variant = 'FNSimple2D';

% (May 27 02:13) Currently the grid and obstacle setup occurs in the python
% file, however, we can potentially make the matrix here, save it to a .txt
% and read it on python to maintain all of our setup in one place.


% %% Run Python code to obtain waypoints
% %-------------------------------------------------------------------------%
% % Python Dependencies: Python 3.8, NumPy, and Matplotlib                  %
% %-------------------------------------------------------------------------%
% 
% % Marco's Directory:
% commandStr = 'python "X:\Marco\Documents\Stanford\Classes\Spring 2020\AA 203 - Optimal Controls\Final Project\AA203_FinalProject\solve_rrt_star.py"';
% 
% % Tom's Directory:
% % commandStr = 
% 
% % Wills' Directory
% % commandStr = 
% 
% [status, commandOut] = system(commandStr);
% 
% if status ~= 0
%     error('PYTHON EXECUTION FAILED')
% end
% 
% % Import Waypoints from resulting text file:
% 
% waypoints = readmatrix('RRTs_waypoints.txt');   % TODO: (May 27 02:05) Current points are placeholders. RRT* Optimization in the works

%% Matlab RRT* Waypoints

result = rrt_star(map, max_iter, is_benchmark, rand_seed, variant);
path_idx = result.getResultantPath();
path = zeros(length(path_idx), 2);
path(:,1) = result.tree(1,path_idx);
path(:,2) = result.tree(2,path_idx);

% success = false;
% while ~success
%     success = true;
%     n = length(path(:,1));
%     for i = 2:n-1
%         
%     end
% end

%% Polynomial Fit







