function path = GetPath(q_p1, q_p2, TN)
    if nargin == 1
        path = ComputePath(1, q_p1);
    else
        if TN == 1
            path1 = ComputePath(1, q_p1);
            path2 = ComputePath(2, q_p2);
        else
            path1 = ComputePath(1, q_p2);
            path2 = ComputePath(2, q_p1);
        end
        path = [path1, path2(:,end-1:-1:1)];
    end
end


function path = ComputePath(TN, q_last_p)
% q_last_p is pointer at the last node in tree, since path is generated from the end    
    global tree;
    
    p = q_last_p;
    path = [];
    while p > 0
        path = [path, tree.nodes{TN}(:,p)];
        p = tree.pointers{TN}{p}(1);
    end
    path = path(:,end:-1:1);
end