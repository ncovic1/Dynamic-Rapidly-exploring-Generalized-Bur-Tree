function [collision, distance, D_opt, S_opt] = DistanceLineSegToPoint(A, B, C)
    collision = false;
    D_opt = C;
    
    t_opt = (C-A)'*(B-A)/((B-A)'*(B-A));
    if t_opt <= 0
        distance = norm(C-A);
        S_opt = A;
    elseif t_opt >= 1
        distance = norm(C-B);
        S_opt = B;
    else
        S_opt = A + t_opt*(B-A);
        distance = norm(C-S_opt);
        if distance < 1e-6
            collision = true;
            return;
        end
    end

end
