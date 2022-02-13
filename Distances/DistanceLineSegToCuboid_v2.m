function [collision, d_c, plane] = DistanceLineSegToCuboid_v2(A, B, obstacle)
    collision = false;
    plane = zeros(6,1);
    
    n = B-A;
    H = [2,       0,       0,       -2*n(1);
         0,       2,       0,       -2*n(2);
         0,       0,       2,       -2*n(3);
         -2*n(1), -2*n(2), -2*n(3), 2*norm(n)^2];
    f = 2*[-A(1); -A(2); -A(3); A'*n];
    LB = [obstacle(1:3); 0];
    UB = [obstacle(4:6); 1];
%     options = optimset('Display','off', 'TolFun',1e-4);
%     [x_min, f_min] = quadprog(H, f, [], [], [], [], LB, UB, [], options);
    x_min = qps_mq(H, f, LB, UB);
    f_min = 0.5*x_min'*H*x_min + f'*x_min;
    d_c = sqrt(f_min+norm(A)^2);
    if d_c < 1e-6
        collision = true;
        return;
    end

    P1 = x_min(1:3);
    P2 = A+n*x_min(4);
    plane = [P1; P2-P1];
end
