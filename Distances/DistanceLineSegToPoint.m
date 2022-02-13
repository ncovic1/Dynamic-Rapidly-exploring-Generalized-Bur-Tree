function [collision, d_c, plane] = DistanceLineSegToPoint(A, B, C)
    collision = false;
    P1 = C;
    
    t_opt = (C-A)'*(B-A)/((B-A)'*(B-A));
    if t_opt <= 0
        d_c = norm(C-A);
        P2 = A;
    elseif t_opt >= 1
        d_c = norm(C-B);
        P2 = B;
    else
        P2 = A + t_opt*(B-A);
        d_c = norm(C-P2);
        if d_c < 1e-6
            collision = true;
            return;
        end
    end
    plane = [P1; P2-P1]; 

end
