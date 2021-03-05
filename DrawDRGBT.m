function graphics = DrawDRGBT(graphics, q_p, q_curr, q_next, N_h, p_h, Q_h_reached, predefined_path, realised_path)
    global robot obstacles;
    
    NN = 1;
    for jj = 1:length(graphics{NN})
        delete(graphics{NN}{jj});
    end
    if robot.dim == 2
        if robot.N_DOF == 2
            subplot(1,2,1);
        end
        [xyz, ~] = DirectKinematics(robot, q_curr);
        for jj = 1:robot.N_links 
            graphics{NN}{2*jj-1} = plot([xyz(1,jj),xyz(1,jj+1)], [xyz(2,jj),xyz(2,jj+1)], 'r','LineWidth',3); hold on;
            graphics{NN}{2*jj} = plot(xyz(1,jj),xyz(2,jj),'Color','k','Marker','.','MarkerSize',20); hold on;
        end
    else
        graphics{NN} = DrawRobot(robot, q_curr, 'red', 0.3);
    end

    NN = 2;
    graphics{NN} = DrawPath(robot, predefined_path, graphics{NN}, [0.7,0.7,0.7]);
    graphics{NN+1} = DrawPath(robot, realised_path, graphics{NN+1}, 'g');
    if robot.N_DOF == 2
        subplot(1,2,2); 
        plot([q_p(1),q_curr(1)],[q_p(2),q_curr(2)],'Color','g','LineWidth',4); hold on;
    end

    NN = 4;
    [graphics_WS_new, graphics_CS_new] = DrawObstacles(obstacles, robot);
%     [graphics_WS_new, graphics_CS_new] = DrawObstacles(obstacles, robot, 0.15);

    for i = 1:length(graphics{NN})
        delete(graphics{NN}{i});
    end
    for i = 1:length(graphics{NN+1})
        delete(graphics{NN+1}{i});
    end
    graphics{NN} = graphics_WS_new;
    graphics{NN+1} = graphics_CS_new;        

    if robot.N_DOF == 2
        NN = 6;   
        subplot(1,2,2);
        delete(graphics{NN});
        graphics{NN} = plot(q_curr(1),q_curr(2),'r.','MarkerSize',20); hold on;

        NN = 7;  
        for jj = 1:length(graphics{NN})
            delete(graphics{NN}{jj});             
            delete(graphics{NN+1}{jj});            
        end        
        for jj = 1:N_h
            if p_h(jj) <= 0
                p_h_temp = 1;
            else
                p_h_temp = ceil(20*p_h(jj));
            end
            try
                graphics{NN}{jj} = plot(Q_h_reached(1,jj),Q_h_reached(2,jj),'bo','MarkerSize', p_h_temp); hold on;  
            catch
            end
            graphics{NN+1}{jj} = plot([q_curr(1),Q_h_reached(1,jj)],[q_curr(2),Q_h_reached(2,jj)],'Color','red','LineWidth',0.5); hold on;
        end 

        NN = 9;        
        delete(graphics{NN});
        graphics{NN} = plot(q_next(1),q_next(2),'rx','MarkerSize',20,'LineWidth',2);
        subplot(1,2,1);     
    end

    drawnow;    
end