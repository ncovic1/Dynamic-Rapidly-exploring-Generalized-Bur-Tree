function [] = DrawDetails(path, eps)
    global robot tree;
    path_mod = [];
    k = 1;
    for i = 1:size(path,2)-1
        q1 = path(:,i);
        q2 = path(:,i+1);
        j = 0;
        D = norm(q2-q1);
        while true            
            path_mod(:,k) = q1 + j*eps*(q2-q1)/D;
            if norm(q2-path_mod(:,k)) <= eps
                path_mod(:,k+1) = q2;
                k = k+1;
                break;
            end
            k = k+1;
            j = j+1;
        end        
    end
    
    if robot.dim == 2
        for i = 1:size(path_mod,2)
            DrawRobot(path_mod(:,i), [0.7,0.7,0.7], 0.2, 1, 1);
        end
    else
        for i = 1:size(path_mod,2)
            DrawRobot(path_mod(:,i), [0.7,0.7,0.7], 0.2);
        end
    end        
    
    if robot.N_DOF == 2
        if ~iscell(tree.nodes)
            nodes{1} = tree.nodes;
            pointers{1} = tree.pointers;
        else
            nodes = tree.nodes;
            pointers = tree.pointers;
        end
        subplot(1,2,2);
        for S = 1:length(nodes)
            if S == 1
                color = {'red','blue.'};
            elseif S == 2
                color = {'blue','red.'};
            else
                color = {'yellow','cyan.'};
            end
            for i = 1:size(nodes{S},2)
                q1 = nodes{S}(:,i);
                for j = 2:length(pointers{S}{i})
                    q2 = nodes{S}(:,pointers{S}{i}(j));
                    plot([q1(1),q2(1)],[q1(2),q2(2)],'Color',color{1},'LineWidth',0.5); hold on;
                    plot(q2(1),q2(2),color{2},'MarkerSize',3.5); hold on;
                end
                plot(q1(1),q1(2),color{2},'MarkerSize',3.5); hold on;
            end
        end   
        
        for i = 1:size(path_mod,2)  
            if i < size(path_mod,2) 
                q1 = path_mod(:,i);
                q2 = path_mod(:,i+1);
                plot([q1(1),q2(1)],[q1(2),q2(2)],'Color',[0.7,0.7,0.7],'LineWidth',4); hold on;
            end
        end
    end
    
end