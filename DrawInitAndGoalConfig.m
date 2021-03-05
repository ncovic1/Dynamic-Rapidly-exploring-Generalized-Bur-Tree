function [] = DrawInitAndGoalConfig(robot)
    L = sum(robot.DH_table(:,1:2),2);
    [xyz_init, ~] = DirectKinematics(robot, robot.q_init);
    [xyz_goal, ~] = DirectKinematics(robot, robot.q_goal);
    if robot.dim == 2
        subplot(1,2,1);
        for i = 1:robot.N_links    % init
            plot([xyz_init(1,i),xyz_init(1,i+1)], [xyz_init(2,i),xyz_init(2,i+1)], 'Color','magenta','LineWidth',3); hold on;
            plot(xyz_init(1,i),xyz_init(2,i),'Color','k','Marker','.','MarkerSize',20); hold on;
        end      
        for i = 1:robot.N_links    % goal
            plot([xyz_goal(1,i),xyz_goal(1,i+1)], [xyz_goal(2,i),xyz_goal(2,i+1)], 'Color','green','LineWidth',3); hold on;
            plot(xyz_goal(1,i),xyz_goal(2,i),'Color','k','Marker','.','MarkerSize',20); hold on;
        end
        grid on; xlabel('$x$'); ylabel('$y$'); axis equal; axis(sum(L)*[-1, 1, -1, 1]);
        if robot.N_DOF == 2 
            subplot(1,2,2);
            plot(robot.q_init(1),robot.q_init(2),'magenta.','MarkerSize',20); hold on; 
            plot(robot.q_goal(1),robot.q_goal(2),'green.','MarkerSize',20); hold on;
            grid on; xlabel('$\theta_1$'); ylabel('$\theta_2$'); axis equal; axis([-pi, pi, -pi, pi]); 
        end
    else
        DrawRobot(robot, robot.q_init, 'magenta', 0.3);
        DrawRobot(robot, robot.q_goal, 'green', 0.3);
        grid on; xlabel('$x$'); ylabel('$y$'); zlabel('$z$'); axis equal; axis(sum(L)*[-1, 1, -1, 1, -1, 1]);
    end
end