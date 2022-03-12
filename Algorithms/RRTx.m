function ALG = RRTx(eps_, N_max_)

global robot obstacles node_pos parent edge_wt cost_goal neighbours line_handles LMC r_radius ...
    discovered_obstacles goal_handles children start_idx temp_edge_wt queue curr_node prev_node ...
    obstacle_cost curr_ptr eps delta graphics draw;

robot.q_init = robot.q_init';
robot.q_goal = robot.q_goal';
eps = eps_;
delta = 0.001;
N_max = N_max_;   %N_max = {1500, 2500, 3500}
draw = 1;

node_pos = NaN(N_max,robot.N_DOF);
parent = NaN(N_max,1);
edge_wt = cell(N_max,1);
temp_edge_wt = edge_wt;
neighbours = cell(N_max,1);
cost_goal = Inf(N_max,1);
LMC = Inf(N_max,1);
children = cell(N_max,1);
queue = Inf(N_max,3);
                     
start_chosen = 0;
start_achieved = 0;
node_pos(1,:) = robot.q_goal;
cost_goal(1) = 0;     
LMC(1) = 0;                         
discovered_obstacles = [];
obstacle_cost = Inf;
start_idx = NaN;

line_handles = gobjects(N_max,1);       
goal_handles = gobjects(N_max,1);     
curr_ptr = gobjects(robot.N_DOF,1);        
graphics = {{line(0,0)}, {line(0,0)}, {line(0,0)}}; 

ALG.costs = Inf;
ALG.T_alg = tic;
i = 2;  % 'i' is the Node number
disp('Generating initial tree is in the progress ...');
while isnan(start_idx)
    r_radius = shrinkingBallRadius(i);
    disp(i);
    
    while true
        if ~start_chosen && rand < 0.1
            rand_point = robot.q_init;
        else
            rand_point = randomNode();
        end
        rand_point = saturate(rand_point);
        collision = extend(rand_point, i, r_radius);
        if ~collision            
            rewireNeighbours(i);
            reduceInconsistency();
            break;
        end
    end  
        
%   If the goal has been reached, plot the goal line
    if rand_point == robot.q_init 
        ALG.i_start = i;
        start_idx = i;
        start_achieved = 1;
        start_chosen = 1;
        previous_goal_cost = cost_goal(start_idx);
        if draw
            plotFrom(start_idx, 0);
        end
    end
    
%   If the cost to goal has decreased, then replot the goal line
    if  start_achieved && (cost_goal(start_idx) < previous_goal_cost)
        previous_goal_cost = cost_goal(start_idx);
        if draw   
            delete(goal_handles);
            plotFrom(start_idx, 0);          
        end
    end
    
    if start_achieved
        ALG.costs = [ALG.costs, cost_goal(start_idx)];
    else
        ALG.costs = [ALG.costs, Inf];
    end
    
    i = i + 1;
end
if ~start_achieved
    disp('Initial path is not found.');
    ALG.i = i;
    return;        
else
    disp('Initial tree has been generated.');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here the algorithm runs dynamically
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
curr_node = start_idx;                      
prev_node = curr_node;
ALG.path = node_pos(curr_node,:);
ALG.collision = false;
ALG.T_iter = 0;
ii = 1;
while true
    T_start = tic;
    r_radius = shrinkingBallRadius(i);
    obstacles = UpdateObstacles(robot, obstacles);    
    
    r_dist = pdist2(node_pos, node_pos(curr_node,:), 'euclidean');
%     r_dist = pdist2(node_pos, node_pos(curr_node,:),'seuclidean',1./[sqrt(w1),sqrt(w2)]);
    in_range = find(r_dist < r_radius);   
    obstacle_inRange = [];
    for index = in_range'
        collision = CheckEdge(node_pos(curr_node,:)', node_pos(index,:)');
        if collision
            obstacle_inRange = [obstacle_inRange; index];
        end
    end

    if any(obstacle_inRange)
        addNewObstacle(obstacle_inRange);
        propogateDescendants();
        verifyQueue(curr_node);
        reduceInconsistency_v2();
    end   
    
    updateRobot();
    ALG.path = [ALG.path; node_pos(curr_node,:)];
    
    if draw 
        Draw();           
    end
        
    collision = CheckCollision(node_pos(curr_node,:));
    if collision
        ALG.T_alg = toc(ALG.T_alg);
        ALG.collision = true;
        disp('Collision !!!');
        break;
    end    
    
    if node_pos(curr_node,:) == robot.q_goal
        ALG.T_alg = toc(ALG.T_alg);
        disp('Goal configuration has been successfully reached!');
        break;
    end
    
    while true
        rand_point = normrnd(node_pos(curr_node,:), r_radius);
        rand_point = saturate(rand_point);
        collision = extend(rand_point, i, r_radius);
        if ~collision
            rewireNeighbours(i);
            reduceInconsistency();
            i = i + 1;
            break;
        end
    end  
    
    ALG.costs = [ALG.costs, cost_goal(start_idx)];
    ALG.T_iter = ((ii-1)*ALG.T_iter + toc(T_start))/ii;
    ii = ii + 1;
    
    if i > 1.02*N_max
        disp('Previse cvorova');
        break;
    end
end
ALG.i = i;
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Subroutines:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rand_point = randomNode()
    global robot;
    rand_point = ((robot.range(:,2)-robot.range(:,1)).*rand(robot.N_DOF,1) + robot.range(:,1))';  % Adding random node in C-space
end


function rand_point = saturate(rand_point)
    global node_pos eps
    
%   Calculate the distance of random point to all the nodes
    dist_points = pdist2(node_pos, rand_point, 'euclidean');
    [n_dist, idx] = min(dist_points);
    
%   Bring the node at a distance of epsilon near to the closest node
    n_point = node_pos(idx,:);
    min_dist = min(n_dist, eps); % If the node is closer than epsilon, we have the same distance
    rand_point = n_point + (rand_point - n_point)*min_dist/n_dist;
    
end


function collision = extend(rand_point, i, radius)
    global robot node_pos LMC parent cost_goal neighbours edge_wt temp_edge_wt line_handles children draw;
    
    node_pos(i,:) = rand_point;

%   After bringing the node closer, now calculate the distances to all the nodes again
    all_edge_wt = pdist2(node_pos, rand_point, 'euclidean');
    n_node_idx = all_edge_wt < radius & all_edge_wt > 0 ;

%   Select the nodes that are within the ball radius. These become the neighbours. The 'n_' represents neighbours 
    n_nodes = find(n_node_idx);
    n_edge_wt = all_edge_wt(n_node_idx);
    n_LMC = LMC(n_node_idx);

%   Find the node with the smallest 'LMC'
    n_total_cost = n_edge_wt + n_LMC;
    [p_cost, p_idx] = min(n_total_cost);
    
%   Check for the collision
    collision = CheckEdge(node_pos(i,:)', node_pos(n_nodes(p_idx),:)');
    if collision
        node_pos(i,:) = NaN;
        return;
    end 
    
%   Initialize the node properties
    parent(i,1) = n_nodes(p_idx);
    cost_goal(i,1) = Inf;
    LMC(i,1) = p_cost;
    neighbours(i,:) = {n_nodes};
    edge_wt(i,:) = {n_edge_wt};

%   Now we have to tell the random points' neighbours that random point is
%   now a neighbour, also add the corresponding edge weight
    for j = 1:length(n_nodes)
        neighbours(n_nodes(j)) = {[neighbours{n_nodes(j)};i]};
        edge_wt(n_nodes(j)) = {[edge_wt{n_nodes(j)};n_edge_wt(j)]};
    end
    temp_edge_wt = edge_wt;

%   Plot line from random point to its parent
    if robot.N_DOF == 2 && draw
        subplot(1,2,2); plot(rand_point(1),rand_point(2),'r.','MarkerSize',5); drawnow;
        handle = line([node_pos(parent(i),1) node_pos(i,1)],[node_pos(parent(i),2) node_pos(i,2)],'Color','b'); hold on;
        line_handles(i) = handle;
    end

%   Get all the children in a separate variable (cell)
    for k = 1:i  
        children(k,:) = {find(parent==k)};
    end    
end

    
function r_radius = shrinkingBallRadius(i)    
    global robot;
    
    eta = sqrt(robot.N_DOF);
    mi = (2*pi)^robot.N_DOF;
    zeta = pi^(robot.N_DOF/2)/gamma(robot.N_DOF/2+1);
    gamma_rrt = 2*((1+1/robot.N_DOF)*(mi/zeta))^(1/robot.N_DOF);
    r_radius = min([gamma_rrt*(log(i)/i)^(1/robot.N_DOF), eta]);
end


function updateRobot()
    global node_pos parent curr_node prev_node draw;
    
    next_node = parent(curr_node);
    if ~isnan(next_node)
        collision = CheckEdge(node_pos(curr_node,:)', node_pos(next_node,:)');
    else
        collision = 1;
    end
    if ~collision
        prev_node = curr_node;
        curr_node = next_node;
        if draw            
            plotFrom(curr_node, 1);
        end
        
    end
    if draw
        plotIndication();
    end    
    
end


function addNewObstacle(nodes)
    global neighbours obstacle_cost edge_wt parent temp_edge_wt;

    for i = 1:length(nodes)
        node = nodes(i);
        edge_wt(node) = {temp_edge_wt{node} + obstacle_cost};
        its_neighbours = neighbours{node};
        for j = 1:length(its_neighbours)
           neigh = its_neighbours(j);
           idx = find(neighbours{neigh}==node);
           t_edge = edge_wt{neigh};       
           t_edge(idx) = temp_edge_wt{neigh}(idx) + obstacle_cost;
           edge_wt(neigh) = {t_edge};
           if parent(neigh)==node
                verifyOrphan(neigh);
           end
        end
    end    
end


function propogateDescendants()
    global discovered_obstacles children neighbours parent cost_goal LMC line_handles draw;

    nodes = discovered_obstacles;
    all_children = [];
    tic;
    while ~isempty(nodes) && toc < 2
        nodes = unique(cell2mat(children(nodes)));  
        all_children = union(all_children,nodes);
    end

    discovered_obstacles = union(discovered_obstacles,all_children);

    for i = 1:length(discovered_obstacles)
        spl_nodes = setdiff(neighbours{discovered_obstacles(i)},discovered_obstacles);
        if ~isempty(spl_nodes)
            cost_goal(spl_nodes) = Inf;
            verifyQueue(spl_nodes); 
        end
    end

    cost_goal(discovered_obstacles) = Inf;
    LMC(discovered_obstacles) = Inf;
    for i = 1:length(discovered_obstacles)
        its_parent = parent(discovered_obstacles(i));
         if ~isnan(its_parent)
             child = children{its_parent};
             child_less = child(child ~= discovered_obstacles(i));
             children(its_parent) = {child_less};
         end
    end
    if draw            
        delete(line_handles(discovered_obstacles));
    end    
    parent(discovered_obstacles) = NaN;
    discovered_obstacles = [];

end


function verifyOrphan(node)
    global queue discovered_obstacles;

    if ~isempty(find(queue(:,1)==node,1))
        queue(queue(:,1)==node,:) = Inf;
    end
    discovered_obstacles = union(discovered_obstacles,node);

end


function verifyQueue(nodes)
    global queue;

    key = getKeys(nodes);
    len = length(nodes);
    for i=1:len
        idx = find(queue(:,1)==nodes(i)); 
        if ~isempty(idx)
            queue(idx,2:3) = key(i,:);
        else
            insertNodes(nodes(i))
        end
    end

end


function insertNodes(nodes)
    global queue;

    key = getKeys(nodes);
    queue(end-length(nodes)+1:end,:) = [nodes,key];
    queue = sortrows(queue,[2 3 1]);

end


function key = getKeys(nodes)
    global LMC cost_goal;

    key = [min([cost_goal(nodes),LMC(nodes)],[],2), cost_goal(nodes)];

end


function top_value = popQueue()
    global queue;

    top_value = queue(1,:);
    queue(1,:) = Inf;
    queue = circshift(queue,-1);

end


function top_value = topQueue()
    global queue;
    top_value = queue(1);

end


function updateLMC(node)
    global neighbours LMC edge_wt parent discovered_obstacles children;

    neigh = neighbours{node};
    edge = edge_wt{node};
    banned_nodes = union(discovered_obstacles,neigh(parent(neigh)==node));
    [~,idx] = setdiff(neigh,banned_nodes);
    neigh = neigh(idx);
    edge = edge(idx);

    [updated_LMC,new_parent] = min(edge + LMC(neigh));
    LMC(node) = updated_LMC;
    parent(node) = neigh(new_parent);
    children(neigh(new_parent)) = {unique([children{neigh(new_parent)};node])};
end


function rewireNeighbours(node)
    global robot cost_goal LMC delta neighbours edge_wt parent line_handles node_pos children draw;

    if cost_goal(node)-LMC(node) > delta || isnan(cost_goal(node)-LMC(node))
        neigh = neighbours{node};
        edge = edge_wt{node};
        idx = parent(neigh) ~= node;
        neigh = neigh(idx);
        edge = edge(idx);

        index = LMC(neigh) > edge + LMC(node);
        to_update = neigh(index);
        jj = 1;
        ind = [];
        for j = 1:length(index)
            if index(j)
                collision = CheckEdge(node_pos(node,:)', node_pos(to_update(jj),:)');
                if collision
                    index(j) = false;
                    ind = [ind, jj];
                end
                jj = jj + 1;
            end
        end
        to_update(ind) = [];
        
        LMC(to_update) = edge(index) + LMC(node);
        parent(to_update) = node;
        children(node) = {unique([children{node}; to_update])};
        
        if robot.N_DOF == 2 && draw
            subplot(1,2,2);
            delete(line_handles(to_update));
            for j = 1:length(to_update)
               handle = line([node_pos(to_update(j),1) node_pos(node,1)],[node_pos(to_update(j),2) node_pos(node,2)],'Color','b');
               line_handles(to_update(j)) = handle;
            end
        end

        further_index = (cost_goal(to_update) - LMC(to_update) > delta) | (isnan(cost_goal(to_update) - LMC(to_update)));
        verifyQueue(to_update(further_index));

    end
end


function reduceInconsistency()    
    global queue cost_goal LMC delta;

    while any(any(~isinf(queue)))
       top = popQueue();

       if cost_goal(top(1)) - LMC(top(1)) > delta || isnan(cost_goal(top(1)) - LMC(top(1)))
            updateLMC(top(1));
            rewireNeighbours(top(1));      
       end
       cost_goal(top(1)) = LMC(top(1));

    end
end


function reduceInconsistency_v2()    
    global queue cost_goal LMC delta curr_node;

    while any(any(~isinf(queue))) && (keyLess(topQueue(),curr_node) || LMC(curr_node)~=cost_goal(curr_node) || cost_goal(curr_node)==Inf)
       top = popQueue();

       if cost_goal(top(1)) - LMC(top(1)) > delta || isnan(cost_goal(top(1)) - LMC(top(1)))
            updateLMC(top(1));
            rewireNeighbours(top(1));      
       end
       cost_goal(top(1)) = LMC(top(1));

    end
end


function value = keyLess(top,start)
    top_key = getKeys(top);        
    start_key = getKeys(start);
    value = top_key(1)<start_key(1) & top_key(2)<start_key(2);

end


function [q_new, reached, collision] = GenerateEdge(q, q_e)
    % Edge is generated from q towards q_e for eps
    % q_new is new reached node
    % collision means whether collision occured when moving from q towards q_e
    % reached means whether q_e is reached
    
    global eps;
    D = norm(q_e-q);
    if D < eps  
        reached = true;
        if D == 0
            collision = false;
            q_new = q;
            return;
        end              
        step = D;
    else
        reached = false;
        step = eps;
    end

    eps0 = eps/10;
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


function collision = CheckEdge(q1, q2)
    collision = false;
    reached = false;
    while ~collision && ~reached
        [q1, reached, collision] = GenerateEdge(q1, q2);
    end
end


function plotIndication()
    global robot node_pos curr_node r_radius curr_ptr;
    
    if robot.N_DOF == 2
        subplot(1,2,2);
        delete(curr_ptr);    
        curr_ptr(1) = plot(node_pos(curr_node,1),node_pos(curr_node,2),'r.','MarkerSize',20); hold on;
        curr_ptr(2) = viscircles(node_pos(curr_node,:),r_radius,'LineStyle','--','Color','r', 'LineWidth',0.5); drawnow;
    end
end


function path = GetPath(i)
    global node_pos parent;
    
    path = node_pos(i,:)';
    while i > 1
        i = parent(i);
        path = [path, node_pos(i,:)'];
    end
end


function plotFrom(iter, state)
    global robot node_pos curr_node prev_node goal_handles;
         
    if robot.N_DOF == 2
        if state
            subplot(1,2,2);   
            if ~isnan(curr_node)
                line([node_pos(curr_node,1) node_pos(prev_node,1)],[node_pos(curr_node,2) node_pos(prev_node,2)],'Color','g','LineWidth',4);
            end
        end
        
        delete(goal_handles);
        path = GetPath(iter);
        for k = 1:size(path,2)-1
            goal_handles(k) = line([path(1,k), path(1,k+1)],[path(2,k), path(2,k+1)],'color',[0.7,0.7,0.7],'LineWidth',4);
        end
    end
end


function Draw()   
    global robot node_pos curr_node graphics
    
    NN = 1;
    [graphics_WS, graphics_CS] = DrawObstacles(0.1);
%     [graphics_WS, graphics_CS] = DrawObstacles();

    for i = 1:length(graphics{NN})
        delete(graphics{NN}{i});
    end
    for i = 1:length(graphics{NN+1})
        delete(graphics{NN+1}{i});
    end
    graphics{NN} = graphics_WS;
    graphics{NN+1} = graphics_CS;        

    NN = 3;
    for jj = 1:length(graphics{NN})
        delete(graphics{NN}{jj});
    end
    
    if robot.dim == 2
        graphics{NN} = DrawRobot(node_pos(curr_node,:), 'red', 0.3, 3, 20);
    else
        graphics{NN} = DrawRobot(node_pos(curr_node,:), 'red', 0.2);
    end       

    drawnow;    
end
