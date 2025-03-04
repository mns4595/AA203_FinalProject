function [final_times, xTraj, yTraj, optimization_history] = Optimize_Time_Ratio(waypoints, initial_times, polyOrder, K_t)
%% A function to find the optimal ratio of times that minimizes the snap cost
% The paper minimizes snap cost summed with a cost related to the overall
% time. Given that the overall time will be scaled by the actuator
% constraint, this part is ignored for the time sweep method.

optimization_method = 1; % Parametric sweep from tail end with update after each time segment is swept
% optimization_method = 2; % Gradient descent option CURRENTLY NOT IMPLEMENTED

%% Optimizing the ratio of times by sweeping each combination of times
% This is done for analysis of the paper results rather than optimizing
% computation time

optimization_history = [{'Cost'}, {'Segment times'}, {'Total Time'}, {'xTraj'},{'yTraj'}];

if optimization_method == 1
    %% Method 3 optimizes tail segment and works backwards
    % test different values parametrically (assuming no coupled terms)
    % Not a fail-safe method, but minimizes run-time and complexity given
    % that we have an unknown time vector length
    
    %% Tuning Parameters
    % Can hard code this in if need be; commented out though
%     K_t = 5000;
    
    %% Initialization
    new_times = initial_times;
    times = new_times + 3;
    iterations = 0;
    total_sweep_intervals = 10;
    sweep_range = 1; % seconds
    total_times_array{1,1} = initial_times;
    [~,~,~,~,cost] = TrajOpt(waypoints,initial_times,polyOrder);
    cost_iteration_array = [cost+ K_t*sum(initial_times)];
    optimization_history = [{'Cost'}, {'Segment times'}, {'Total Time'}, {'xTraj'},{'yTraj'}];
    
    % Tuning Loop
    while norm(times - new_times) >= 1e-4 && iterations < 30
        iterations = iterations + 1;
        times = new_times; % Reference of previous iteration's times
        sweep_times = new_times; % Times that will be updated in following for loop
        sweep_results = [{'cost'}, {'time'}];
        
        for segment_ind = length(times):-1:1
            sweep_results = [{'cost'}, {'time'}];
            sweep_results{segment_ind+1,1} =[];
            sweep_results{segment_ind+1,2} =[];
            for sweep_step = 0:total_sweep_intervals
                current_time_value = times(segment_ind) - sweep_range/2 + sweep_step / total_sweep_intervals * sweep_range;
                if current_time_value > 0
                    sweep_times(segment_ind)= current_time_value;
                    [~,~,~,~,cost] = TrajOpt(waypoints,sweep_times,polyOrder);
                    % Each row represents a segment
                    time_cost = cost + K_t * current_time_value;
                    sweep_results{segment_ind+1,1} = [ sweep_results{segment_ind+1,1} time_cost];
                    sweep_results{segment_ind+1,2} = [ sweep_results{segment_ind+1,2} current_time_value];
                    % doesn't check value when t is less than 0 
                end
            end
            
            % Updates the segment based on the minimum time
            [~,lowest_cost_ind] = min(sweep_results{segment_ind+1, 1});
            sweep_times(segment_ind) = sweep_results{segment_ind+1,2}(lowest_cost_ind);
        end
        
        total_times_array{1,iterations + 1} = sweep_times;
        [~,~,xTraj,yTraj,new_cost] = TrajOpt(waypoints,sweep_times,polyOrder); % with code optimization, this line could be removed
        total_new_cost = new_cost + K_t * sum(sweep_times);
        cost_iteration_array(iterations + 1, 1) = total_new_cost;
        
        if cost_iteration_array(end)>cost_iteration_array(end-1) && iterations > 5
            % A check in case cost is not decreasing
            [~,min_ind] =min(cost_iteration_array);
            new_times =  total_times_array{1,min_ind};
            disp('Error: the cost has increased on this iteration; times reverted to previous value. Cost is wrong in the logging arrays.');
        else
            % Normal Operation
            new_times = sweep_times;
        end
        
        % Update logging cell array
        optimization_history{iterations+1,1} = total_new_cost;
        optimization_history{iterations+1,2} = new_times;
        optimization_history{iterations+1,3} = sum(new_times);
        optimization_history{iterations+1,4} = xTraj;
        optimization_history{iterations+1,5} = yTraj;
    end
    final_times = new_times;
    final_cost = total_new_cost;
    disp('Iteration ' + string(iterations) + ' complete');

end
disp('The initial guess of times for this optimization were: ');
disp(initial_times')
disp('The initial total time was ' + string(sum(initial_times)) + 's');
disp('The final set of times for this optimization are: ');
disp(final_times');
disp('The final total time was ' + string(sum(final_times)) + 's');
disp('The cost of the final trajectory is: ');
disp(final_cost);

end