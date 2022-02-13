function path = GetPath(q_p1, q_p2, TN)
    if TN == 1
        path1 = ComputePath(1, q_p1);
        path2 = ComputePath(2, q_p2);
    else
        path1 = ComputePath(1, q_p2);
        path2 = ComputePath(2, q_p1);
    end
    path = [path1(:,end:-1:2), path2];
end
