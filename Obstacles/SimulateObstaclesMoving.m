close all;
clear all;
global robot obstacles;

robot.dim           = 3;        % Robot dimensionality in workspace
robot.model         = 2;        % Model of robotic manipulator (See details in 'Initialization')
obstacles.model     = 2;        % Model of obstacles (1: spherical, 2: boxed)
obstacles.type      = 5;        % See details bellow
obstacles.step      = 0.1;      % obstacles moves by the size of this step
obstacles.delta_t   = 0.050;
obstacles.max_vel   = 1.0;
Initialization;

fig = figure();
fig.WindowState = 'maximized';
DrawRobot(robot.q_init, 'magenta', 0.3, 3, 20);     % The library 'geom3d' must be included for 3D robot. 
DrawRobot(robot.q_goal, 'green', 0.3, 3, 20);       % You can find it at https://github.com/mattools/matGeom
graphics = {{line(0,0)}, {line(0,0)}};

tic;
while toc < 60
    [graphics_WS_new, graphics_CS_new] = DrawObstacles(0.03, 6);
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