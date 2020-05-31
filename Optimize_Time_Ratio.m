function [new_times, xTraj, yTraj] = Optimize_Time_Ratio(waypoints, initial_times, polyOrder)
new_times = initial_times;

%%%% TO DO %%%%%%
% Perform Optimization, minimizing cost

    [~,~,xTraj,yTraj,~] = TrajOpt(waypoints,new_times,polyOrder);
end

