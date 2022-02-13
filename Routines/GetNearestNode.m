function [q_near, q_near_p] = GetNearestNode(nodes, q)
    q_near_p = 1;
    d_min = inf;
    for i = 1:size(nodes,2)
        q_temp = nodes(:,i);
        is_out = false;
        for k = 1:length(q)
            if abs(q_temp(k)-q(k)) > d_min    % Is outside the box?
                is_out = true;
                break;
            end
        end
        if ~is_out   % Is inside the box?
            d = norm(q_temp-q);
            if d < d_min
                q_near_p = i;
                d_min = d;
            end
        end
    end
    q_near = nodes(:,q_near_p);
end