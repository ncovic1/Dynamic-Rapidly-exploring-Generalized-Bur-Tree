classdef RRT
properties
    eps = 0.1;          % Step in C-space used by RRT-based algorithms
    eps0 = 0.01;        % Step in C-space for collsion check by RRT-based algorithms
    N_max = 10000;      % Max. number of considered nodes
    p = 0.1;            % Probability of choosing robot.q_goal
    tree;               % Consisting of two parts, one from q_init and another from robot.q_goal. E.g. [q1,q11,q12,q121,q13,q122,q14,q123,q1231,q1232]
    pointers = {0};     % Pointing to location of children in tree. First column is parent and others are children. E.g. {[0,2,3,5,7],[1],[1,4,6,8],[3],[1],[3],[1],[3,9,10],[8],[8]}
    path = [];          % Traversed path (sequence of nodes from q_init to robot.q_goal)
    N_nodes = 2;        % Number of considered nodes in tree
    T_alg = 0;          % Total algorithm runtime  
end

methods
    function this = RRT(eps, eps0, N_max, p)
        if nargin > 0
            this.eps = eps;
            this.eps0 = eps0;
            this.N_max = N_max;
            this.p = p;
        end
    end
    
    
    function this = Run(this)
        global robot obstacles;
        
        this.T_alg = tic;
        [collision, ~, ~] = CheckCollision(robot, obstacles, robot.q_init);
        if collision
            disp('Initial robot configuration is in collision!');
            return;
        end        
        [collision, ~, ~] = CheckCollision(robot, obstacles, robot.q_goal);
        if collision
            disp('Goal robot configuration is in collision!');
            return;
        end      
        this.tree = robot.q_init;
        
        while true
            if rand < this.p
                q_rand = robot.q_goal;
            else
                q_rand = 2*pi*rand(robot.N_DOF,1)-pi;  % Adding random node in C-space
            end
            [q_near, q_near_p] = Find_q_near(this.tree, q_rand);
            [q_new, ~, collision] = this.GenerateEdge(q_near, q_rand);
            if ~collision
                [this.tree, this.pointers, q_new_p] = UpdateTree(this.tree, this.pointers, q_near_p, q_new);
                this.N_nodes = this.N_nodes + 1;
                if q_new == robot.q_goal
                    this.path = GetPath(this.tree, this.pointers, q_new_p);    
                    this.T_alg = toc(this.T_alg); 
                    disp(['Path is found in ', num2str(this.T_alg), ' [s].']);
                    return;
                end
            end
            if this.N_nodes >= this.N_max
                this.T_alg = toc(this.T_alg);
                disp('Path is not found.');
                return;
            end
        end
    end
    
    
    function [q_new, reached, collision, d_c] = GenerateEdge(this, q, q_e)
        % Edge is generated from q towards q_e for eps
        % q_new is new reached node
        % collision means whether collision occured when moving from q towards q_e
        % reached means whether q_e is reached
        % d_c is minimal distance to obstacles
        global robot obstacles;
        
        q_new = q;
        D = norm(q_e-q);
        if D < this.eps
            reached = true;
            step = D;
            K_max = ceil(step/this.eps0);
        else
            reached = false;
            step = this.eps;
            K_max = ceil(this.eps/this.eps0);
        end
        if D == 0            
            [collision, d_c, ~] = CheckCollision(robot, obstacles, q_new);
            return;
        end
        
        collision = false;
        smjer = step*(q_e-q)/D;
        for k = 1:K_max
            q_new = q + k/K_max*smjer;
            [collision, d_c, ~] = CheckCollision(robot, obstacles, q_new);
            if collision
                q_new = q;
                collision = true;
                reached = false;
                break;
            end
        end  
    end
    
end
end