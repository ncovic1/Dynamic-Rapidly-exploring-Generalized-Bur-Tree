function path = GetPath(tree, pointers, q_last_p)
    % q_last_p is pointer at the last node in tree, since path is generated from end
    
    i = q_last_p;
    path = tree(:,i);
    while true
        i = pointers{i}(1);
        if i > 0
            path = [path, tree(:,i)];
        else
            break;
        end
    end
    path = path(:,end:-1:1);
end