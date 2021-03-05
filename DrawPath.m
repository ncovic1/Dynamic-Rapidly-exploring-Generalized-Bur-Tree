function graphics = DrawPath(robot, path, graphics, color)

    if robot.N_DOF == 2
        subplot(1,2,2);
        for i = 1:length(graphics)
            delete(graphics{i});
        end               
        for i = 1:size(path,2)  
            if i < size(path,2) 
                x = path(:,i);
                y = path(:,i+1);
                graphics{i} = plot([x(1),y(1)],[x(2),y(2)],'Color',color,'LineWidth',4); hold on;
            end
        end
    else
        graphics = {};
    end
end