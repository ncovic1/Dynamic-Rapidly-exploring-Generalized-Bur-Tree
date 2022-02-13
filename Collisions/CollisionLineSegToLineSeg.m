function collision = CollisionLineSegToLineSeg(A, B, C, D)
    global robot;
    
    collision = false;    
    alpha1 = (B-A)'*(B-A);
    alpha2 = (B-A)'*(D-C);
    beta1 = (C-D)'*(B-A);
    beta2 = (C-D)'*(D-C);
    gamma1 = (A-C)'*(A-B);
    gamma2 = (A-C)'*(C-D);
    s = (alpha1*gamma2-alpha2*gamma1)/(alpha1*beta2-alpha2*beta1);
    t = (gamma1-beta1*s)/alpha1;    
    if t >= 0 && t <= 1 && s >= 0 && s <= 1
        if robot.dim == 2
            collision = true;
        else
            P1 = C + s*(D-C);
            P2 = A + t*(B-A);
            if norm(P2-P1) < 1e-6
                collision = true;
            end
        end
    end 
        
end