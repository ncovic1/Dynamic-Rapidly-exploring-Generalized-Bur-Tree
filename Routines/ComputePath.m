function path = ComputePath(TN, q_last_p)
    % q_last_p is pointer at the last node in tree, since path is generated from the end    
    global tree;
    
    path = [];
    while q_last_p > 0
        path = [path, tree.nodes{TN}(:,q_last_p)];
        q_last_p = tree.pointers{TN}{q_last_p}(1);
    end
end