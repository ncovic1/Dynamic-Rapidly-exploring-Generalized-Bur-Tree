function [collision, distance, D_opt, S_opt] = DistanceLineSegToLineSeg(A, B, C, D)
    global robot d_c_temp D_temp S_temp;
    
    collision = false;
    distance = inf;    
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
            distance = 0;
            S_opt = zeros(3,1);
            D_opt = zeros(3,1); 
        else
            S_opt = A + t*(B-A);
            D_opt = C + s*(D-C);
            distance = norm(S_opt-D_opt);
            if distance < 1e-6
                collision = true;
            end
        end
    else    
        var1 = 1/((B-A)'*(B-A));
        var2 = 1/((C-D)'*(C-D));
        ConsiderCases(1, (A-C)'*(A-B)*var1);  % s = 0
        ConsiderCases(2, (A-D)'*(A-B)*var1);  % s = 1
        ConsiderCases(3, (A-C)'*(D-C)*var2);  % t = 0
        ConsiderCases(4, (B-C)'*(D-C)*var2);  % t = 1
    end
    
    function ConsiderCases(Case, opt)
        % Case = [1,2,3,4] = [s=0, s=1, t=0, t=1]
        
        if opt <= 0
            if Case == 1 || Case == 3       % s = 0, t = 0
                D_temp = C; S_temp = A;
            elseif Case == 2                % s = 1, t = 0
                D_temp = D; S_temp = A;
            else                            % t = 1, s = 0
                D_temp = C; S_temp = B;
            end
        elseif opt >= 1                             
            if Case == 2 || Case == 4       % s = 1, t = 1
                D_temp = D; S_temp = B;
            elseif Case == 1                % s = 0, t = 1
                D_temp = C; S_temp = B;
            else                            % t = 0, s = 1
                D_temp = D; S_temp = A;
            end
        else
            if Case == 1                    % s = 0, t € [0,1]
                D_temp = C; S_temp = A+opt*(B-A);
            elseif Case == 2                % s = 1, t € [0,1]
                D_temp = D; S_temp = A+opt*(B-A);
            elseif Case == 3                % t = 0, s € [0,1]
                D_temp = C+opt*(D-C); S_temp = A;
            else                            % t = 1, s € [0,1]
                D_temp = C+opt*(D-C); S_temp = B;                              
            end
        end
        
        d_c_temp = norm(D_temp-S_temp);
        if d_c_temp < distance
            D_opt = D_temp; S_opt = S_temp;
            distance = d_c_temp;
        end
    end

end
