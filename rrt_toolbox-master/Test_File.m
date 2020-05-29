%% Test File

map = struct('name', 'poly_world.mat', 'start_point', [0.5 0.5], 'goal_point', [9 16]);
max_iter =  2000;
is_benchmark = false;
rand_seed = 40;
variant = 'FNSimple2D';
result = rrt_star(map, max_iter, is_benchmark, rand_seed, variant);