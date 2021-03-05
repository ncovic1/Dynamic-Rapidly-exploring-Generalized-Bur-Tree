function [] = DrawDetails(Tree, Pointers, path, robot)
    if robot.N_DOF == 2
        if ~iscell(Tree)
            tree{1} = Tree;
            pointers{1} = Pointers;
        else
            tree = Tree;
            pointers = Pointers;
        end
        subplot(1,2,2);
        for S = 1:length(tree)
            if S == 1
                color = {'red','blue.'};
            elseif S == 2
                color = {'blue','red.'};
            else
                color = {'yellow','cyan.'};
            end
            for i = 1:size(tree{S},2)
                q1 = tree{S}(:,i);
                for j = 2:length(pointers{S}{i})
                    q2 = tree{S}(:,pointers{S}{i}(j));
                    plot([q1(1),q2(1)],[q1(2),q2(2)],'Color',color{1},'LineWidth',0.5); hold on;
                    plot(q2(1),q2(2),color{2},'MarkerSize',3.5); hold on;
                end
                plot(q1(1),q1(2),color{2},'MarkerSize',3.5); hold on;
            end
        end        
    end
    
    path_mod = [];
    eps = 0.09;
    k = 1;
    for i = 1:size(path,2)-1
        q1 = path(:,i);
        q2 = path(:,i+1);
        j = 0;
        while true            
            path_mod(:,k) = q1 + j*eps*(q2-q1)/norm(q2-q1);
            if norm(q2-path_mod(:,k)) <= eps
                path_mod(:,k+1) = q2;
                break;
            end
            k = k+1;
            j = j+1;
        end        
    end
    
    if robot.dim == 2
        for i = 1:size(path_mod,2)  
            if robot.N_DOF == 2 && i < size(path_mod,2)                
                subplot(1,2,2);
                q1 = path_mod(:,i);
                q2 = path_mod(:,i+1);
                plot([q1(1),q2(1)],[q1(2),q2(2)],'Color',[0.7,0.7,0.7],'LineWidth',4); hold on;
            end
            [xyz, ~] = DirectKinematics(robot, path_mod(:,i));
            if i > 1
                subplot(1,2,1);
                if i < size(path_mod,2)
                    for j = 1:robot.N_links      
                        plot([xyz(1,j),xyz(1,j+1)], [xyz(2,j),xyz(2,j+1)], 'Color',[0.7,0.7,0.7],'LineWidth',1); hold on; %drawnow;
                    end
                end
                plot([xyz(1,end),xyz_p(1)], [xyz(2,end),xyz_p(2)], 'Color','red','LineWidth',1); hold on;
            end
            xyz_p = [xyz(1,end); xyz(2,end)];
        end
    else
        for i = 1:size(path_mod,2)  
%             [xyz, ~] = DirectKinematics(robot, path_mod(:,i));
%             if i > 1
%                 if i < size(path_mod,2)
%                     for j = 1:robot.N_links      
%                         plot3([xyz(1,j),xyz(1,j+1)], [xyz(2,j),xyz(2,j+1)], [xyz(3,j),xyz(3,j+1)], 'Color',[0.7,0.7,0.7],'LineWidth',0.5); hold on;
%                     end
%                 end
%                 plot3([xyz(1,end),xyz_p(1)], [xyz(2,end),xyz_p(2)], [xyz(3,end),xyz_p(3)], 'Color','red','LineWidth',1); hold on;
%             end
%             xyz_p = xyz(:,end);
            r=0.1;        
            [~] = DrawRobot(robot, path_mod(:,i), [0.7,0.7,0.7], r, 0.1);
        end
    end
    
end