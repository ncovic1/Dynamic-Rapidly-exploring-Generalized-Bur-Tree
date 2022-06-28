function [graphics_WS, graphics_CS] = DrawObstacles(d_theta, MarkerSize)
    global robot obstacles;
    graphics_WS = {};
    graphics_CS = {};
    L = sum(robot.DH_table(:,1:2),2);
    if robot.dim == 2
        if robot.N_DOF == 2
            subplot(1,2,1);
        end
        if obstacles.model == 1
            theta = linspace(-pi,pi,100);
            for i = 1:size(obstacles.loc,2)
                x_p = obstacles.loc(1,i) + obstacles.loc(4,i)*cos(theta);
                y_p = obstacles.loc(2,i) + obstacles.loc(4,i)*sin(theta);
                graphics_WS{i} = fill(x_p, y_p, 'k'); hold on;
            end
        elseif obstacles.model == 2
            for i = 1:size(obstacles.loc,2)
                x_p = [obstacles.loc(1,i), obstacles.loc(4,i), obstacles.loc(4,i), obstacles.loc(1,i)];
                y_p = [obstacles.loc(2,i), obstacles.loc(2,i), obstacles.loc(5,i), obstacles.loc(5,i)];
                graphics_WS{i} = fill(x_p, y_p, 'k'); hold on;
            end
        end
        grid on; xlabel('$x$'); ylabel('$y$'); axis equal; axis(sum(L)*[-1, 1, -1, 1]);

        if robot.N_DOF == 2 && nargin == 2
            i = 1;
            subplot(1,2,2); 
            for theta1 = -pi:d_theta:pi
                for theta2 = -pi:d_theta:pi
                    collision = CheckCollision([theta1; theta2]);
                    if collision
                        graphics_CS{i} = plot(theta1, theta2,'black','Marker','.','MarkerSize',MarkerSize); hold on; %drawnow; 
                        i = i + 1;
                    end
                end
            end
            grid on; xlabel('$\theta_1$'); ylabel('$\theta_2$'); axis equal; axis([robot.range(1,:), robot.range(2,:)]);
        end
    else
        if obstacles.model == 1
            fi = -pi:0.1:pi;
            k = 1;
            for i = 1:size(obstacles.loc,2)
                for theta = -pi:0.05:pi
                    x_p1 = obstacles.loc(1,i) + obstacles.loc(4,i)*cos(fi)*sin(theta);
                    y_p1 = obstacles.loc(2,i) + obstacles.loc(4,i)*sin(fi)*sin(theta);
                    z_p1 = obstacles.loc(3,i) + obstacles.loc(4,i)*cos(theta)*ones(1,length(fi));
                    theta = theta + 0.05;
                    x_p2 = obstacles.loc(1,i) + obstacles.loc(4,i)*cos(fi)*sin(theta);
                    y_p2 = obstacles.loc(2,i) + obstacles.loc(4,i)*sin(fi)*sin(theta);
                    z_p2 = obstacles.loc(3,i) + obstacles.loc(4,i)*cos(theta)*ones(1,length(fi));
                    graphics_WS{k} = fill3([x_p1,x_p2], [y_p1,y_p2], [z_p1,z_p2], 'k'); hold on;  
                    k = k + 1;
                end
            end          
        elseif obstacles.model == 2
            k = 1;
            for i = 1:size(obstacles.loc,2)
                x1 = obstacles.loc(1,i); y1 = obstacles.loc(2,i); z1 = obstacles.loc(3,i);
                x2 = obstacles.loc(4,i); y2 = obstacles.loc(5,i); z2 = obstacles.loc(6,i);
                graphics_WS{k} =   fill3([x1,x2,x2,x1], [y1,y1,y2,y2], [z1,z1,z1,z1], 'k'); hold on;
                graphics_WS{k+1} = plot3([x1,x2,x2,x1,x1], [y1,y1,y2,y2,y1], [z1,z1,z1,z1,z1], 'w'); hold on;
                graphics_WS{k+2} = fill3([x1,x2,x2,x1], [y1,y1,y2,y2], [z2,z2,z2,z2], 'k'); hold on;
                graphics_WS{k+3} = plot3([x1,x2,x2,x1,x1], [y1,y1,y2,y2,y1], [z2,z2,z2,z2,z2], 'w'); hold on;
                graphics_WS{k+4} = fill3([x1,x2,x2,x1], [y1,y1,y1,y1], [z1,z1,z2,z2], 'k'); hold on;
                graphics_WS{k+5} = plot3([x1,x2,x2,x1,x1], [y1,y1,y1,y1,y1], [z1,z1,z2,z2,z1], 'w'); hold on;
                graphics_WS{k+6} = fill3([x1,x2,x2,x1], [y2,y2,y2,y2], [z1,z1,z2,z2], 'k'); hold on;
                graphics_WS{k+7} = plot3([x1,x2,x2,x1,x1], [y2,y2,y2,y2,y2], [z1,z1,z2,z2,z1], 'w'); hold on;
                graphics_WS{k+8} = fill3([x1,x1,x1,x1], [y1,y2,y2,y1], [z1,z1,z2,z2], 'k'); hold on;
                graphics_WS{k+9} = plot3([x1,x1,x1,x1,x1], [y1,y2,y2,y1,y1], [z1,z1,z2,z2,z1], 'w'); hold on;
                graphics_WS{k+10} = fill3([x2,x2,x2,x2], [y1,y2,y2,y1], [z1,z1,z2,z2], 'k'); hold on;
                graphics_WS{k+11} = plot3([x2,x2,x2,x2,x2], [y1,y2,y2,y1,y1], [z1,z1,z2,z2,z1], 'w'); hold on;
                k = k+12;
            end
        end
        grid on; xlabel('$x$'); ylabel('$y$'); zlabel('$z$'); axis equal; axis(sum(L)*[-1, 1, -1, 1, -1, 1]);  
    end
end