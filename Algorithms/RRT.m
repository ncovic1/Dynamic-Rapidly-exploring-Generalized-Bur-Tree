classdef RRT
properties
    eps = 0.1;          % Step in C-space used by RRT-based algorithms
    N_max = 10000;      % Max. number of considered nodes
    p = 0.1;            % Probability of choosing robot.q_goal
    path = [];          % Traversed path (sequence of nodes from q_init to robot.q_goal)
    N_nodes = 2;        % Number of considered nodes in trees
    T_alg = 0;          % Total algorithm runtime  
end

methods
    function this = RRT(eps, N_max, p)
        if nargin > 0
            this.eps = eps;
            this.N_max = N_max;
            this.p = p;
        end
    end
    
    
    function this = Run(this)
        global robot tree;        
        TN = 1; % Tree number is always 1 for RRT
        tree.nodes = {robot.q_init};  % List of nodes in tree. E.g. [q1,q11,q12,q121,q13,q122,q14,q123,q1231,q1232]
        tree.pointers = {{0}};        % Pointing to location of parent/children in trees. First column is parent and others are children. E.g. {[0,2,3,5,7],[1],[1,4,6,8],[3],[1],[3],[1],[3,9,10],[8],[8]}
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
            if rand < this.p
                q_rand = robot.q_goal;
            else
                q_rand = this.GetRandomNode();
            end
            [q_near, q_near_p] = GetNearestNode(tree.nodes{TN}, q_rand);
            [q_new, ~, collision] = this.GenerateEdge(q_near, q_rand);
            if ~collision
                q_new_p = UpgradeTree(TN, q_near_p, q_new);
                this.N_nodes = this.N_nodes + 1;
                if q_new == robot.q_goal
                    this.path = ComputePath(1, q_new_p);
                    this.path = this.path(:,end:-1:1);
                    this.T_alg = toc(this.T_alg); 
                    disp(['The path is found in ', num2str(this.T_alg), ' [s].']);
                    return;
                end
            end
            if this.N_nodes >= this.N_max
                this.T_alg = toc(this.T_alg);
                disp('The path is not found.');
                return;
            end
        end
    end
    
    
    function q_e = GetRandomNode(~)
        % Adding a random node with uniform distribution in C-space
        global robot;
        
        q_e = (robot.range(:,2)-robot.range(:,1)).*rand(robot.N_DOF,1) + robot.range(:,1);
    end
    
    
    function [q_new, reached, collision] = GenerateEdge(this, q, q_e)
        % Edge is generated from q towards q_e for eps
        % q_new is new reached node
        % collision means whether collision occured when moving from q towards q_e
        % reached means whether q_e is reached
        
        D = norm(q_e-q);
        if D < this.eps  
            reached = true;
            if D == 0
                collision = false;
                q_new = q;
                return;
            end              
            step = D;
        else
            reached = false;
            step = this.eps;
        end
        
        eps0 = this.eps/10;
        K = ceil(step/eps0);
        Step = step/(K*D);
        for k = K:-1:1
            q_new = q + k*Step*(q_e-q);                
            collision = CheckCollision(q_new);
            if collision
                q_new = q;
                reached = false;
                return;
            end
        end
        q_new = q + K*Step*(q_e-q); 
        
    end
    
end
end