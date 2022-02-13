classdef RRT_Connect < RRT
properties        

end

methods
    function this = RRT_Connect(eps, N_max)
        if nargin > 0
            this.eps = eps;
            this.N_max = N_max;
        end
    end
    
    
    function this = Run(this)
        global robot tree;
        TN = 2;  % Determines which trees is chosen, 1: from q_init; 2: from q_goal
        tree.nodes = {robot.q_init, robot.q_goal};  % Consisting of two parts, one from q_init and another from q_goal
        tree.pointers = {{0}, {0}};     % Pointing to location of parent/children in trees
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
            % Edge is generated
            TN = 3 - TN;
            q_rand = this.GetRandomNode();
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_rand);
            [q_new, ~, collision] = this.GenerateEdge(q_near, q_rand);
            if ~collision
                q_new_p = UpgradeTree(TN, q_near_p, q_new);
                this.N_nodes = this.N_nodes + 1; 
            else
                q_new_p = q_near_p;
            end
            
            % Two trees are trying to connect
            TN = 3 - TN;
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_new);
            collision = false;  % Whether the collision occured
            reached = false;    % Whether trees are connected
            while ~collision && ~reached
                [q_near, reached, collision] = this.GenerateEdge(q_near, q_new);
                if ~collision
                    q_near_p = UpgradeTree(TN, q_near_p, q_near);
                    this.N_nodes = this.N_nodes + 1; 
                end 
            end
            
            if reached
                this.path = GetPath(q_near_p, q_new_p, TN);
                this.T_alg = toc(this.T_alg);
                disp(['The path is found in ', num2str(this.T_alg), ' [s].']);
                return;
            elseif this.N_nodes >= this.N_max
                this.T_alg = toc(this.T_alg);
                disp('The path is not found.');
                return;
            end
            TN = 3 - TN;
        end
    end
    
end
end