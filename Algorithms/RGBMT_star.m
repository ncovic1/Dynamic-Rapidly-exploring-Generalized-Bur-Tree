classdef RGBMT_star < RGBT
properties
    N_nodes = [1, 1];       % Total number of nodes in all trees
    cost_opt = inf;         % The cost of the final path
    costs = [];             % Best cost through iterations
    return_WPF = false;     % Whether to return When Path is Found (default: false)
end

methods    
    function this = RGBMT_star(eps, N_max, d_crit, T_max)
        if nargin > 0
            this.eps = eps;
            this.N_max = N_max;
            this.d_crit = d_crit;
            this.T_max = T_max;
        end
    end
    
    
    function this = Run(this)
        global robot tree writerObj;
        
        tree.nodes = {robot.q_init, robot.q_goal};  % Consisting of two parts, one from q_init and another from q_goal
        tree.pointers = {{0}, {0}};                 % Pointing to location of parent/children in trees
        tree.distances = {NaN, NaN};                % Distance to each obstacle for each node
        tree.planes = {{{NaN}}, {{NaN}}};           % Lines/planes dividing space into two subspaces (free and "occupied")
        tree.costs = {0, 0};                        % Cost-to-come for each node in corresponding tree
        TN_new = 3;                                 % Index of the new tree
        this.T_alg = tic;
        
        if CheckCollision(robot.q_init)
            disp('Initial robot configuration is in the collision!');
            return;
        end        
        if CheckCollision(robot.q_goal)
            disp('Goal robot configuration is in the collision!');
            return;
        end
        
        while true
%             disp([num2str((this.N_nodes(1) + this.N_nodes(2))), ' ', num2str(sum(this.N_nodes) - (this.N_nodes(1) + this.N_nodes(2)))]);
            q_rand = this.GetRandomNode();            
                        
            %% Adding a new tree to 'tree'
            [d_c, planes] = GetDistance(q_rand);
            tree.nodes{TN_new} = q_rand;
            tree.pointers{TN_new} = {0};
            tree.distances{TN_new} = d_c;
            tree.planes{TN_new} = {planes};
            tree.costs{TN_new} = 0;
            trees_reached_p = zeros(1,TN_new-1);    % Pointers to reached nodes in other trees
            trees_reached = [];                     % List of reached trees
            trees_exist = [];                       % List of trees for which a new tree is extended to
            cost_update = false;

            %% Considering all previous trees
            for TN = 1:TN_new-1     
                % If the connection with q_near is not possible, attempt to connect with parent(q_near), etc.
                [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_rand);                
                while true      
                    [reached, q_new] = this.ConnectNodes(TN_new, 1, q_near);    % q_rand to q_near
                    q_parent_p = tree.pointers{TN}{q_near_p}(1);
                    if reached || q_parent_p == 0
                        break;
                    else
                        q_near = tree.nodes{TN}(:,q_parent_p);
                        q_near_p = q_parent_p;
                    end
                end              

                %% Whether currently considering tree (TN_new-th tree) is reached
                cost = this.GetCost(q_new, q_rand);
                if reached      % If reached, q_new == q_near
                    UpgradeTree(TN_new, 1, q_new, tree.distances{TN}(q_near_p), tree.planes{TN}{q_near_p}, cost);
                    trees_reached_p(TN) = q_near_p;
                    trees_exist = [trees_exist, TN];
                    trees_reached = [trees_reached, TN];
                elseif cost >= this.eps     % Edge shorter than 'eps' is not considered
                    UpgradeTree(TN_new, 1, q_new, NaN, NaN, cost);
                    trees_exist = [trees_exist, TN];
                end
            end
            
%             this.DrawLocalTrees();
%             this.Draw_CS();
%             pause(0.000000025);
%             writeVideo(writerObj, getframe(gcf));

            %% Find the optimal edge towards each reached tree
            if ~isempty(trees_reached)
                %% The connection of q_rand with both main trees exists
                TN0 = trees_reached(1);  % Considering the first reached tree
                if length(trees_reached) > 1 && prod(trees_reached(1:2) == [1,2])   % Both main trees are reached
                    if rand > this.N_nodes(2)/(this.N_nodes(1)+this.N_nodes(2))
                        TN0 = trees_reached(2);     % q_rand will be joined to the second main tree
                    end
                end

                %% Considering all edges from the new tree
                q_rand_p = this.OptimizeEdge(TN_new, 1, TN0, trees_reached_p(TN0)); % From q_rand to tree TN0
                k = 1;  % Index of the node from the new tree TN_new
                trees_connected = [];
                for TN = trees_exist
                    k = k+1;
                    if TN == TN0  % It was considered previously, so just skip now
                        continue;
                    end

                    %% Unification of tree TN with TN0. Main trees are never unified mutually
                    q_new_p = this.OptimizeEdge(TN_new, k, TN0, q_rand_p);  % From k-th reached node to tree TN0  
                    if TN > 2 && any(TN == trees_reached)    
                        this.UnifyTrees(TN, trees_reached_p(TN), TN0, q_new_p);
                        trees_connected = [trees_connected, TN];

                    %% The connection of q_rand with both main trees exists
                    elseif TN < 3 && length(trees_reached) > 1 && prod(trees_reached(1:2) == [1,2])
                        q_parent_p = tree.pointers{TN}{trees_reached_p(TN)}(1); 
                        while q_parent_p > 0
                            q_new_p = this.OptimizeEdge(TN, q_parent_p, TN0, q_new_p);
                            q_parent_p = tree.pointers{TN}{q_parent_p}(1);
                        end
                        cost = tree.costs{TN0}(q_new_p);
                        if cost < this.cost_opt     % The optimal connection between main trees is stored
                            this.cost_opt = cost;
                            disp(['Cost after ', num2str(sum(this.N_nodes)), ' nodes is ', num2str(cost)]);
                            q_con_p = q_new_p; 
                            TN_main = TN0;          % Index of the considered main tree when the connection of two main trees occured 
                            cost_update = true;
                        end
                    end
                end
%                 this.Draw_CS();
%                 pause(0.000000025);
%                 writeVideo(writerObj, getframe(gcf));
                
                %% Deleting trees that have been connected
                trees_connected = [trees_connected, TN_new];
                this.DeleteTrees(trees_connected);
                TN_new = TN_new - length(trees_connected) + 1;

            else    % If there are no reached trees, then the new tree is added to 'tree'
%                 this.Draw_CS();
%                 pause(0.000000025);
%                 writeVideo(writerObj, getframe(gcf));
                
                TN_new = TN_new + 1;
            end
            
            N_nodes_prev = sum(this.N_nodes);
            this.N_nodes = 0;
            for TN = 1:TN_new-1
                this.N_nodes(TN) = size(tree.nodes{TN},2);
            end
            this.costs = [this.costs, this.cost_opt*ones(1,sum(this.N_nodes)-length(this.costs))];
            
            %%%%%%%%%%%% Drawing %%%%%%%%%%%%%%%
%             if sum(this.N_nodes) > N_nodes_prev
%                 if this.cost_opt < inf && cost_update
%                     this.path = ComputePath(TN_main, q_con_p);
%                     this.Draw_WS();
%                 end                
%                 pause(0.000000025);
%                 writeVideo(writerObj, getframe(gcf));
%             end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if this.return_WPF && this.cost_opt < inf || sum(this.N_nodes) >= this.N_max
                if this.cost_opt < inf     % Both main trees are connected
                    this.path = ComputePath(TN_main, q_con_p);
                    this.T_alg = toc(this.T_alg);
                    disp(['The path is found in ', num2str(this.T_alg), ' [s] with the cost of ', num2str(this.cost_opt), '.']);
                else
                    this.T_alg = toc(this.T_alg);
                    disp('The path is not found.');
                end
                return;
            end
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function q_rand = GetRandomNode(this)
        global robot tree;
        
        alpha = 1;
        while true
            q_rand = (robot.range(:,2)-robot.range(:,1)).*rand(robot.N_DOF,1) + robot.range(:,1);  % Uniform distribution
            
%             if length(tree.nodes) > 2   % Gaussian distribution
%                 [N_main, TN] = min(N_nodes(1:2));
%                 N_local = sum(N_nodes(3:end));
%                 if N_local > N_main
%                     q_rand = normrnd(tree.nodes{TN}(:,1), (robot.range(:,2)-robot.range(:,1))*N_main/(6*N_local));
%                     for i = 1:robot.N_DOF
%                         if q_rand(i) < robot.range(i,1)
%                             q_rand(i) = robot.range(i,1);
%                         elseif q_rand(i) > robot.range(i,2)
%                             q_rand(i) = robot.range(i,2);
%                         end
%                     end
%                 end
%             end

            if sum(this.N_nodes) - (this.N_nodes(1) + this.N_nodes(2)) > alpha * (this.N_nodes(1) + this.N_nodes(2))  % If local trees contain more states than main trees
                %disp('Standard RGBT extension ............ ');
                [~, TN] = min(this.N_nodes(1:2));
                [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_rand);
                [~, q_new] = this.ConnectNodes(TN, q_near_p, q_rand);    % q_near to q_rand
                if ~prod(q_new == q_near)
                    q_rand = q_new;
                    return;
                end
            elseif ~CheckCollision(q_rand)  % If q_rand is collision-free, it is accepted
                return;
            end               
        end
    end
    
    
    function cost = GetCost(~, q1, q2)
        global robot;
        cost = norm(q1-q2);
%         cost = pdist2(q1', q2', 'seuclidean', 1./sqrt(robot.weights));
    end
    
    
    function [reached, q_new] = ConnectNodes(this, TN, q_p, q_e)
        % Generate spine from node (from tree TN and pointer q_p) to q_e
        % q_new is reached node
        
        global tree;
        q = tree.nodes{TN}(:,q_p);
        d_c = tree.distances{TN}(q_p);
        planes = tree.planes{TN}{q_p};
        collision = false;
        reached = false;
        q_new = q;       
             
        while ~collision && ~reached
            % If the distance-to-obstacles is greater/less than d_crit
            if d_c > this.d_crit
                [q_new, reached] = this.GenerateSpine(q_new, q_e, d_c);     % Generating a generalized spine
                d_c = this.GetUnderestimation_dc(q_new, planes);
            else
                [q_new, reached, collision] = this.GenerateEdge(q_new, q_e);     % Generating a spine according to RRT-paradigm
            end
        end
%         plot(q(1),q(2),'r.');   
%         plot([q(1),q_new(1)],[q(2),q_new(2)],'r');
    end
        
    
    function q_p = OptimizeEdge(this, TN, q_p, TN0, q_reached_p)
        % Node q (from tree TN and pointer q_p) is optimally connected to the tree TN0
        % q_reached_p is a pointer to the node to which q is initially connected in tree TN0
        
        global tree;
        
        % Finding the optimal edge to the predecessors of q_parent
        q_parent_p = tree.pointers{TN0}{q_reached_p}(1); 
        while q_parent_p > 0
            [reached, ~] = this.ConnectNodes(TN, q_p, tree.nodes{TN0}(:,q_parent_p));
            if reached
                q_reached_p = q_parent_p;
            end
            q_parent_p = tree.pointers{TN0}{q_parent_p}(1);
        end
        
        q_parent_p = tree.pointers{TN0}{q_reached_p}(1);    % Parent of the reached node
        if q_parent_p > 0
            update = false;
            q1 = tree.nodes{TN0}(:,q_reached_p);      % It is surely collision-free. It will become an optimal node
            q2 = tree.nodes{TN0}(:,q_parent_p);   % Needs to be collision-checked
            D = norm(q1-q2);
            for i = 1:floor(log2(10*D))
                q_middle = (q1+q2)*0.5;
                [reached, ~] = this.ConnectNodes(TN, q_p, q_middle);            
                if reached
                    q1 = q_middle;
                    update = true;
                else
                    q2 = q_middle;
                end
            end
            if update
                cost = tree.costs{TN0}(q_parent_p) + this.GetCost(tree.nodes{TN0}(:,q_parent_p), q1);
                q1_p = UpgradeTree(TN0, q_parent_p, q1, NaN, NaN, cost);
                
                tree.pointers{TN0}{q1_p}(2) = q_reached_p;                      % Adding the child of q1
                tree.pointers{TN0}{q_reached_p}(1) = q1_p;                      % Modifying the parent of q_opt
                ind = find(tree.pointers{TN0}{q_parent_p} == q_reached_p);
                tree.pointers{TN0}{q_parent_p}(ind) = q1_p; %#ok<FNDSB>	% Modifying the child of q_parent
                q_reached_p = q1_p;
            end
        end
        
        cost = tree.costs{TN0}(q_reached_p) + this.GetCost(tree.nodes{TN0}(:,q_reached_p), tree.nodes{TN}(:,q_p));
        q_p = UpgradeTree(TN0, q_reached_p, tree.nodes{TN}(:,q_p), tree.distances{TN}(q_p), tree.planes{TN}{q_p}, cost);        
    end
    
    
    function UnifyTrees(this, TN, q_con_p, TN0, q0_con_p)
        % TN - a local tree that is optimally unified with TN0
        % TN0 - a main tree (or the first reached local tree)
        % q_con_p - Pointer to the node that connects TN with TN0
        % q0_con_p - Pointer to the node that connects TN0 with TN
        
        global tree;
        q_considered_p = 0;
        while true
            ConsiderChildren(q_con_p, q0_con_p, q_considered_p);
            q_parent_p = tree.pointers{TN}{q_con_p}(1);
            if q_parent_p == 0
                break;
            end
            q0_con_p = this.OptimizeEdge(TN, q_parent_p, TN0, q0_con_p);
            q_considered_p = q_con_p;
            q_con_p = q_parent_p;
        end
        
        function ConsiderChildren(q_p, q0_p, q_considered_p)
            % Consider all children (except 'q_considered', that has already being considered) of 'q' from tree 'TN', and connect them optimally to 'tree0' 
        
            children_p = GetAvailableChildren(TN, q_p, q_considered_p);
            for child_p = children_p
                q0_p2 = this.OptimizeEdge(TN, child_p, TN0, q0_p);
                if ~isempty(tree.pointers{TN}{child_p}(2:end))  % child has its children
                    ConsiderChildren(child_p, q0_p2, 0);     % '0' means that no children will be removed
                end
            end
        end
        
        function children_p = GetAvailableChildren(TN, q_p, q_considered_p)
            % Remove child 'q_considered_p' of 'q_p' from tree 'TN'
            
            no_of_children = length(tree.pointers{TN}{q_p})-1;
            if no_of_children > 0  % If children exists
                children_p =  tree.pointers{TN}{q_p}(2:no_of_children+1);
                children_p(children_p == q_considered_p) = [];  % Node that was considered before is not treated as a child   
            else
                children_p = [];
            end
        end
    end
    
    
    function DeleteTrees(~, TNs)
        % Delete all trees with indices (tree numbers) TNs
        global tree;
        
        tree.nodes(TNs) = [];
        tree.pointers(TNs) = [];
        tree.distances(TNs) = [];
        tree.planes(TNs) = [];
        tree.costs(TNs) = [];
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function DrawLocalTrees(~)
        global graphics_local local_trees local_opacity;
        for i = 1:length(graphics_local)
            for j = 1 : length(graphics_local{i})
                delete(graphics_local{i}{j});
            end
        end
        
        for TN = length(local_trees) :-1: 1
            if local_opacity(TN) <= 1e-6
                local_trees(TN) = [];
                local_opacity(TN) = [];
            end
        end
        
        subplot(1,2,2);
        step = 0.2; % step for cooling
        num = 1;
        k = 1;
        color = [0.9290, 0.6940, 0.1250];   % orange
        for TN = 1:length(local_trees)
            for i = 1:size(local_trees{TN}.nodes,2)
                x = local_trees{TN}.nodes(:,i);
                for j = 2:length(local_trees{TN}.pointers{i})
                    y = local_trees{TN}.nodes(:,local_trees{TN}.pointers{i}(j));
                    line = plot([x(1),y(1)],[x(2),y(2)],'Color',color, 'LineWidth',1); hold on;
                    line.Color = [line.Color, local_opacity(TN)];
                    graphics_local{num}{k} = line;
                    k = k+1;
                end
            end
            local_opacity(TN) = local_opacity(TN) - step;
        end
    end
    
    function Draw_CS(this)
        global tree graphics_CS local_trees local_opacity;
        
        for i = 1:length(graphics_CS)
            delete(graphics_CS{i});
        end
        
        subplot(1,2,2);
        k = 1;
        linewidth = 1;
        for TN = 1:length(tree.nodes)
            if TN == 1
                color = [0, 0.4470, 0.7410];    % blue
            elseif TN == 2
                color = [0.8500, 0.3250, 0.0980];   % red
            elseif TN == length(tree.nodes)
                color = [0.4940, 0.1840, 0.5560];   % magenta (for local trees)
                linewidth = 2;
%                 local_tree.nodes = tree.nodes{TN};
%                 local_tree.pointers = tree.pointers{TN};
%                 local_trees{length(local_trees)+1} = local_tree;
%                 local_opacity(length(local_opacity)+1) = 1;
            else
                color = [0.9290, 0.6940, 0.1250]; % orange
            end
            for i = 1:size(tree.nodes{TN},2)
                x = tree.nodes{TN}(:,i);
                for j = 2:length(tree.pointers{TN}{i})
                    y = tree.nodes{TN}(:,tree.pointers{TN}{i}(j));
                    graphics_CS{k} = plot([x(1),y(1)],[x(2),y(2)],'Color',color, 'LineWidth',linewidth); hold on;
                    k = k+1;
                end
            end
        end
        
        title(['Num. of states: ', num2str(sum(this.N_nodes)), '~~~~Cost: ', num2str(this.cost_opt)]);
        if this.cost_opt < inf            
            for i = 1:size(this.path,2)-1
                graphics_CS{k} = line([this.path(1,i), this.path(1,i+1)],[this.path(2,i), this.path(2,i+1)],...
                	'Color',[0.7,0.7,0.7],'LineWidth',5); hold on; 
                k = k+1;
            end
        end
        drawnow;
    end
    
    function Draw_WS(this)
        global graphics_WS;
                
        kk = 1;
        eps = 0.1;
        for i = 1:size(this.path,2)-1
            q1 = this.path(:,i);
            q2 = this.path(:,i+1);
            j = 0;
            D = norm(q2-q1);
            while true
                if D > 0
                    path_mod(:,kk) = q1 + j*eps*(q2-q1)/D;
                else
                    path_mod(:,kk) = q1;
                end
                if norm(q2-path_mod(:,kk)) <= 1.5*eps
                    path_mod(:,kk+1) = q2;
                    kk = kk+1;
                    break;
                end
                kk = kk+1;
                j = j+1;
            end        
        end

        for i = 1:length(graphics_WS)
            for j = 1 : length(graphics_WS{i})
                delete(graphics_WS{i}{j});
            end
        end
        
        k = 1;        
        for i = 1:size(path_mod,2)
            graphics_WS{k} = DrawRobot(path_mod(:,i), [0.7,0.7,0.7], 0.2, 1, 1);
            k = k+1;
%                 drawnow;
        end
        drawnow;
    end
end
end