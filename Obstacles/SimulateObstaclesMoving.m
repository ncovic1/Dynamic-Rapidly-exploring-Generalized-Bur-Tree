close all;
clear all;
global robot obstacles;

Initialization;
DrawInitAndGoalConfig(robot);
graphics = {{line(0,0)}, {line(0,0)}};

tic;
while toc < 10
%     [graphics_WS_new, graphics_CS_new] = DrawObstacles(obstacles, robot, 0.08);
    [graphics_WS_new, graphics_CS_new] = DrawObstacles(obstacles, robot);
    for i = 1:length(graphics{1})
        delete(graphics{1}{i});
    end
    for i = 1:length(graphics{2})
        delete(graphics{2}{i});
    end
    graphics{1} = graphics_WS_new;
    graphics{2} = graphics_CS_new; 
    obstacles = UpdateObstacles(robot, obstacles); drawnow;
%     pause(0.01);
end