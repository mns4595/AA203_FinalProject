function PlotTrajectory(waypoints, obstacles, polyOrder, xTraj, yTraj)

figure('Renderer', 'painters', 'Position', [1920/2 50 1920/3 900])
for i = 1: length(obstacles)
    fill(obstacles{i}(:,1), obstacles{i}(:,2), 'k')
    hold on
end
goal = fill([8 8 10 10 8], [16 18 18 16 16], 'g');
set(goal, 'facealpha', 0.5)
set(goal, 'edgealpha', 0)

if polyOrder == 1
    title('A plot of the straight-line trajectory output from RRT*')
    plot(waypoints(:,1), waypoints(:,2), 'linewidth', 3)
    scatter(waypoints(:,1), waypoints(:,2), 'filled')
    
%     print(gcf,'RRTstar_waypoints.png','-dpng','-r300'); % UNCOMMENT TO SAVE HIGH RESOLUTION
else
    title('A plot of the polynomial trajectory with segments having polynomial order ' + string(polyOrder))
    for i=1:length(waypoints)-1
        plot(xTraj{i},yTraj{i}, 'LineWidth', 3)
        hold on
    end
    scatter(waypoints(:,1),waypoints(:,2), 'linewidth', 1.5)
    
%     print(gcf,'FinalTraj.png','-dpng','-r300'); % UNCOMMENT TO SAVE HIGH RESOLUTION
    
    %% Scatter of trajectory points to show quadrotor speed
    figure('Renderer', 'painters', 'Position', [1920/2 50 1920/3 900])
    for i = 1: length(obstacles)
        fill(obstacles{i}(:,1), obstacles{i}(:,2), 'k')
        hold on
    end
    goal = fill([8 8 10 10 8], [16 18 18 16 16], 'g');
    set(goal, 'facealpha', 0.5)
    set(goal, 'edgealpha', 0)
    title('A plot of the discrete trajectory with using polynomial order ' + string(polyOrder))
    for i=1:length(waypoints)-1
        scatter(xTraj{i},yTraj{i})
        hold on
    end
    scatter(waypoints(:,1),waypoints(:,2))
end

xlim([0 10])
ylim([0 18])

% print(gcf,'FinalTraj_scatter.png','-dpng','-r300'); % UNCOMMENT TO SAVE HIGH RESOLUTION

end
