function [tree, pointers, q_new_p] = UpdateTree(tree, pointers, q_parent_p, q_new)
    % q_parent_p - pointer at q_new parent
    % q_new - new node added to tree
    
    tree = [tree, q_new];
    q_new_p = size(tree,2);
    pointers{q_parent_p} = [pointers{q_parent_p}, q_new_p];  % Determines at which location this parent has child
    pointers{q_new_p} = q_parent_p;  % Determines at which location this child has parent
    
end