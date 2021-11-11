classdef DRGBT < RGBT
properties
    N_h0 = 10;              % Initial horizon size
    Q_h;                    % Nodes from the horizon
    w_h;                    % Node weights
    distance_max_mean = 0;  % Averaged maximal distance-to-obstacles through iterations
    underestimation = [];   % Underestimation of distances-to-obstacle for each node from Q_h
    q_curr;                 % Current robot configuration
    q_next;                 % Next robot configuration
    collision = 0;          % Whether the collision occured
    T_s = inf;              % Maximal runtime of a single DRGBT iteration (also 1/f_s, where f_s is the number of frames per second)
    T_iter = 0;             % Averaged runtime of a single DRGBT iteration
end

methods
    function this = DRGBT(eps, N_max, N_spines, N_layers, d_crit, delta, T_s)
        global robot;
        if nargin > 0
            this.eps = eps;
            this.N_max = N_max;
            this.N_spines = N_spines;
            this.N_layers = N_layers;
            this.d_crit = d_crit;
            this.delta = delta;
            this.T_s = T_s;
        end
        this.delta = pi/2;      % This value is better for DRGBT
        this.N_max = 250*robot.N_DOF+1000;
        this.N_iter = 1;
    end
    
    function this = Run(this)
        global robot obstacles graphics;
        graphics = {{line(0,0)}, {line(0,0)}, {line(0,0)}, {line(0,0)}, {line(0,0)}, line(0,0), {line(0,0)}, {line(0,0)}, line(0,0)};
        
%         global OBS name;
%         for nn = 1:size(obstacles.loc,2)
%             OBS(:,:,nn) = obstacles.loc(:,nn);
%         end
%         writerObj = VideoWriter(['Video', name],'MPEG-4'); % Name it.
%         writerObj.FrameRate = 10; % How many frames per second.
%         writerObj.Quality = 100;  % Quality of the video
%         open(writerObj);
        
        this.T_alg = tic;   % Total DRGBT runtime        
        T1 = inf;           % Maximal runtime for function GenerateGBur is 0.006 [s]        
        k = 1;              % Index of next node from predefined path
        index_next = 1;     % Index of next node from Q_h
        N_h = this.N_h0;
        this.q_curr = robot.q_init;
        this.path = this.q_curr;
        
        % Getting inital path
        rgbt = RGBT();
        rgbt = rgbt.Run();
        
        while true
            T_iter_temp = tic;
            replanning = false;   % Whether path replanning is required
            k = k + index_next;     
            if k+N_h-1 <= size(rgbt.path,2)
                this.Q_h = rgbt.path(:,k:k+N_h-1);
            elseif k <= size(rgbt.path,2)
                this.Q_h = rgbt.path(:,k:end);
            else
                k = size(rgbt.path,2);
                if isempty(rgbt.path)
                    this.Q_h = this.Get_q_e(N_h);  % If initial path was not found, N_h random nodes are added
                else
                    this.Q_h = rgbt.path(:,end);   % At least q_goal remains in horizon
                end
            end
            
            this.q_next = this.Q_h(:,1);
            [Q_h_reached, this.Q_h, this.underestimation, N_h, ind_not_in_path] = this.GenerateGBur(this.Q_h, [], T1);            
            [this.w_h, this.q_next, index_next, this.distance_max_mean] = this.GetWeights(N_h, ind_not_in_path, Q_h_reached, zeros(1,N_h));  % Relative weight for choosing a node
            index_prev = index_next;
            
            if k == size(rgbt.path,2) && ~prod(this.q_next == robot.q_goal)
                replanning = true;
            end
            
            var = false;
            ind_good = 1:N_h;  % Indices of good nodes in horizon
            ind_bad = [];      % Indices of bas nodes in horizon
            reached = false;
            while ~reached           
                obstacles = UpdateObstacles(robot, obstacles);
%                 OBS = [OBS, zeros(6,1,size(obstacles.loc,2))];
%                 for nn = 1:size(obstacles.loc,2)
%                     OBS(:,end,nn) = obstacles.loc(:,nn);
%                 end
                
                this.T_iter = ((this.N_iter-1)*this.T_iter + toc(T_iter_temp))/this.N_iter;
                this.N_iter = this.N_iter + 1;                
                if var
                    T_iter_temp = tic;
                end
                
                D = norm(this.q_next-this.q_curr);
                q_p = this.q_curr;
                if D > this.eps
                    this.q_curr = q_p + this.eps*(this.q_next-q_p)/D;               
                else
                    this.q_curr = this.q_next;
                    reached = true;
                end
                this.path = [this.path, this.q_curr];
                var = true;
                
                this.collision = CheckCollision(this.q_curr);
                if this.collision
                    this.T_alg = toc(this.T_alg);
                    this.Draw(q_p, this.q_curr, this.q_next, N_h, this.w_h, Q_h_reached, rgbt.path, this.path);
                    disp('Collision !!!');
                    
                    pause(0.000000025);
                    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                    writeVideo(writerObj, frame);
                    close(writerObj);
                    return;
                end
                
                if this.q_curr == robot.q_goal
                    this.T_alg = toc(this.T_alg);
                    this.Draw(q_p, this.q_curr, this.q_next, N_h, this.w_h, Q_h_reached, rgbt.path, this.path); 
                    disp('Goal configuration has been successfully reached!');
                    
                    pause(0.000000025);
                    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                    writeVideo(writerObj, frame);
                    close(writerObj);
                    return;
                end
                
                [Q_h_reached, this.Q_h, distance_new, N_h, ind_not_in_path] = this.GenerateGBur(this.Q_h(:,ind_good), this.Q_h(:,ind_bad), T1);
                ind_good = 1:N_h;
                ind_bad = [];
                if length(distance_new) > length(this.underestimation)
                    this.underestimation(end+1:length(distance_new)) = distance_new(length(this.underestimation)+1:length(distance_new));
                elseif length(distance_new) < length(this.underestimation)
                    this.underestimation = this.underestimation(1:length(distance_new));
                end
                delta_dist = distance_new - this.underestimation;   % Change of underestimation
                this.underestimation = distance_new;  
                [this.w_h, this.q_next, index_next, this.distance_max_mean] = this.GetWeights(N_h, ind_not_in_path, Q_h_reached, delta_dist, index_prev);                  
                index_prev = index_next;       
                
                if max(this.w_h) < 0.5 && mean(this.w_h) < 0.5 || replanning  % Condition for replanning
                    T_remain = this.T_s - toc(T_iter_temp) - 0.001;     % Added 1 [ms] for remaining code lines
                    index_next = 1;
                    robot.q_init = this.q_curr;
                    A_new = RGBT(this.eps, this.N_max, this.N_spines, this.N_layers, this.d_crit, this.delta, T_remain);
                    A_new = A_new.Run();

                    if ~isempty(A_new.path)  % Update path to the goal
%                         disp('The path has been replanned.');
                        replanning = false;
                        reached = true;
                        rgbt.path = A_new.path;
                        k = 1;
                    else    % New path is not found
%                         disp('New path is not found.');
                        replanning = true;   
                        ind_good = [];
                        ind_bad = [];
                        for ii = 1:N_h
                            if this.w_h(ii) == 0  % Bad nodes
                                ind_bad = [ind_bad, ii];
                            else
                                ind_good = [ind_good, ii];
                            end
                        end
                    end
                end
                
                % Real-time checking
%                 T_remain = this.T_s - toc(T_start);
% %                 disp(['Time remaining: ', num2str(T_remain*1000), ' [ms]']);
%                 if T_remain < 0
% %                     disp('Real-time is broken !!!');
%                 else
%                     while this.T_s-toc(T_start) > 0
%                     end
%                 end
                
%                 disp('----------------------------------------------------------------------------------------');
            
                this.Draw(q_p, this.q_curr, this.q_next, N_h, this.w_h, Q_h_reached, rgbt.path, this.path); 
                
                pause(0.000000025);
                frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                writeVideo(writerObj, frame);

            end            
        end
    end
    
    
    function [Q_h_reached, Q_h, underestimation, N_h, ind_not_in_path] = GenerateGBur(this, Q_h, Q_h_bad, T_max)
        % Q_h_reached contains reached nodes from horizon
        % Q_h contains good nodes in the beginning, but later it contains updated horizon
        % ind_not_in_path contains indices of horizon nodes that are not in the path
        % Q_h_bad are bad nodes that will be replaced with "better" nodes
        % T_max is maximal runtime for this function
        % underestimation contains underestimation for each node from Q_h_reached
        global robot obstacles;
        
        T = tic;
        [~, d_c, separatrix] = CheckCollision(robot, obstacles, this.q_curr);
        N_h = floor(this.N_h0*(1 + this.d_crit/d_c));
        if N_h > 5*robot.N_DOF*this.N_h0
            N_h = 5*robot.N_DOF*this.N_h0;
        end
        
        Q_h_good_size = size(Q_h,2);
        Q_h_bad_size = size(Q_h_bad,2);
        ind_not_in_path = Q_h_good_size+1:N_h;
        
        if Q_h_good_size > N_h    % Surplus nodes are deleted. Best nodes holds priority.
            Q_h = Q_h(:,1:N_h);
            Q_h_bad = [];
            Q_h_bad_size = 0;
            ind_not_in_path = [];
        elseif Q_h_good_size + Q_h_bad_size >= N_h
            Q_h_bad = Q_h_bad(:,1:N_h-Q_h_good_size);
            Q_h_bad_size = size(Q_h_bad,2);
            Q_h = [Q_h, Q_h_bad];
        else    % If N_h has increased or little nodes exist, random/lateral nodes are added
            Q_h = [Q_h, Q_h_bad];
            if robot.N_DOF == 2
                Q_h(:,Q_h_good_size+Q_h_bad_size+1:N_h) = this.Get_q_e(N_h-Q_h_good_size-Q_h_bad_size);
            else
                Q_h(:,Q_h_good_size+Q_h_bad_size+1:N_h) = this.Get_q_e(N_h-Q_h_good_size-Q_h_bad_size, []);
            end
        end
        
        % Bad nodes are modified
        if Q_h_bad_size > 0
            Q_h(:,Q_h_good_size+1:Q_h_good_size+Q_h_bad_size) = this.Get_q_e(Q_h_bad_size, Q_h_bad, true);
        end
        
        % Lateral spines are added
        N_h_old = N_h;
        N_h = N_h_old + 2*robot.N_DOF - 2;
        Q_h(:,N_h_old+1:N_h) = this.Get_q_e(N_h-N_h_old, []);
        ind_not_in_path = [ind_not_in_path, N_h_old+1:N_h];     % node indices that are not in the path
        
        i = 0;  % Number of added spines
        while i < N_h
            i = i + 1; 
            [q_new, reached] = this.GenerateSpine(this.q_curr, Q_h(:,i), d_c);
            d_c_new = this.Update_d_c(q_new, separatrix); 
                 
            if d_c_new < this.d_crit && ~prod(q_new == robot.q_goal)    % underestimation to obstacle is less than critical
                Q_h(:,i) = this.Get_q_e(1, q_new, false);   % q_e is added instead of Q_h(:,i), which is "bad"
                [q_new, reached] = this.GenerateSpine(this.q_curr, Q_h(:,i), d_c);
                d_c_new = this.Update_d_c(q_new, separatrix); 
                ind_not_in_path(end+1) = i;
            end
            
            for jj = 2:this.N_layers
                if reached   % q_e has been reached     
                    break;
                end
                [q_new, reached] = this.GenerateSpine(q_new, Q_h(:,i), d_c_new); 
                d_c_new = this.Update_d_c(q_new, separatrix);
            end
            underestimation(i) = d_c_new;   % Underestimation for underestimation   
            Q_h_reached(:,i) = q_new; 
            
            if toc(T) > T_max     % Interrupt
                Q_h = Q_h(:,1:i);
                N_h = i;
                if i >= ind_not_in_path(1)
                    ind_not_in_path = ind_not_in_path(1:i-ind_not_in_path(1)+1);
                else
                    ind_not_in_path = [];
                end
                return;
            end
        end
    end    
    
    
    function Q_e = Get_q_e(this, N, q, orientation)
        global robot;
        
        % Adding random spines
        Q_e = repmat(this.q_curr,1,N) + 2*rand(robot.N_DOF,N)-1;

        if nargin == 3  % Adding lateral spines
            if robot.N_DOF == 2   % In 2D C-space only two possible spines exist
                Q_e = [this.q_curr(1), this.q_curr(1); this.q_curr(2)-1, this.q_curr(2)+1];
            end
            if this.q_next(1)-this.q_curr(1) ~= 0
                for i = 1:N
                    Q_e(1,i) = ((this.q_next-this.q_curr)'*this.q_curr - ...
                        (this.q_next(2:end)-this.q_curr(2:end))'*Q_e(2:end,i))/(this.q_next(1)-this.q_curr(1));
                end
            end
        elseif nargin == 4  % Adding random node with oriented weight around q (orientation = true) or around -q (orientation = false)
            for i = 1:N
                a_norm = norm(q(:,i)-this.q_curr);
                temp_set = [-1,1];
                r = (2*rand(robot.N_DOF,1)-1)*a_norm/sqrt(robot.N_DOF-1);
                r(1) = temp_set(randi(2))*sqrt(a_norm^2-norm(r(2:end))^2);
                if orientation
                    Q_e(:,i) = q(:,i) + r;
                else
                    Q_e(:,i) = 2*this.q_curr - q(:,i) + r;
                end
            end
        end

        for i = 1:N
            Q_e(:,i) = this.SaturateSpine(this.q_curr, Q_e(:,i), this.delta);
            Q_e(:,i) = this.PruneSpine(this.q_curr, Q_e(:,i));
        end

    end
    
    
    function [w_h, q_next, index_next, distance_max_mean] = GetWeights(this, N_h, ind_not_in_path, Q_h_reached, delta_dist, index_prev)
        global robot;
        
        distance_max_mean = ((this.N_iter-1)*this.distance_max_mean + max(this.underestimation))/this.N_iter;
        d_i = zeros(1,N_h);
        for i = 1:N_h
            d_i(i) = norm(robot.q_goal-Q_h_reached(:,i));
        end
        index_goal = 0;
        [d_best, index] = min(d_i);  % Minimal underestimation to goal
        if d_best == 0  % q_goal is cosidered
            d_best = 0.00001;  % Added only to avoid 0/0 when d_i(i)==0
            d_i(index) = d_best;
            index_goal = index;
        end
        p_dist = d_best./d_i;
        p_dist_mean = mean(p_dist);
        for i = 1:N_h
            if this.underestimation(i) > this.d_crit
                w_h(i) = (this.underestimation(i) + delta_dist(i))/distance_max_mean + p_dist(i) - p_dist_mean;   
                if w_h(i) > 1
                    w_h(i) = 1;
                elseif w_h(i) < 0
                    w_h(i) = 0;
                end
            else
                w_h(i) = 0;
            end
        end
        w_h(ind_not_in_path) = w_h(ind_not_in_path)/2;  % weight is halved if node does not exist in the path
        
        hysteresis = 0.1;
        [w_h_max, index_next] = max(w_h);
        D_min = inf;
        for i = 1:N_h
            if abs(w_h_max - w_h(i)) < hysteresis
                if d_i(i) < D_min    % The best node nearest to q_goal is chosen
                    D_min = d_i(i);
                    index_next = i;
                end
            end
        end
        if nargin == 6  % If weights of previous and new node are close, previous node remains
            if index_next ~= index_prev && index_prev <= length(w_h)
                if D_min == 0   % If q_goal has been reached, hysteresis is set to zero
                    hysteresis = 0;
                end
                if abs(w_h(index_next) - w_h(index_prev)) < hysteresis
                    index_next = index_prev;
                end
            end
        end        
        q_next = Q_h_reached(:,index_next);
    end
    
    
    function Draw(q_p, q_curr, q_next, N_h, p_h, Q_h_reached, predefined_path, realised_path)
        global robot obstacles graphics;

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
    %     [graphics_WS_new, graphics_CS_new] = DrawObstacles(obstacles, robot);
        [graphics_WS_new, graphics_CS_new] = DrawObstacles(obstacles, robot, 0.03);

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
end
end