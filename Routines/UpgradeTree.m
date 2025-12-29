function q_new_p = UpgradeTree(TN, q_parent_p, q_new, d_c, planes, cost)
    % q_parent_p - pointer at q_new parent in tree TN
    % q_new - new node added to tree
    % q_new_p - pointer at q_new in the new tree
    
    global tree;
    tree.nodes{TN} = [tree.nodes{TN}, q_new];
    q_new_p = size(tree.nodes{TN},2);
    tree.pointers{TN}{q_parent_p} = [tree.pointers{TN}{q_parent_p}, q_new_p];  % Determines at which location this parent has child
    tree.pointers{TN}{q_new_p} = q_parent_p;  % Determines at which location this child has parent
    if nargin == 4
        tree.distances{TN}(q_new_p) = d_c;
    elseif nargin == 5
        tree.distances{TN}(q_new_p) = d_c;
        tree.planes{TN}{q_new_p} = planes;
    elseif nargin == 6
        tree.distances{TN}(q_new_p) = d_c;
        tree.planes{TN}{q_new_p} = planes;
        tree.costs{TN}(q_new_p) = cost;
    end
end