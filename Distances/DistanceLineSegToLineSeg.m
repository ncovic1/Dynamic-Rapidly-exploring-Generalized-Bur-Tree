function [collision, d_c, plane] = DistanceLineSegToLineSeg(A, B, C, D)
    global robot;
    
    collision = false;
    d_c = inf;    
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
            d_c = 0;
            P1 = zeros(3,1); 
            P2 = zeros(3,1);
        else
            P1 = C + s*(D-C);
            P2 = A + t*(B-A);
            d_c = norm(P2-P1);
            if d_c < 1e-6
                collision = true;
            end
        end
        plane = [P1; P2-P1]; 
    else    
        var1 = 1/((B-A)'*(B-A));
        var2 = 1/((C-D)'*(C-D));
        opt = [(A-C)'*(A-B)*var1, (A-D)'*(A-B)*var1, (A-C)'*(D-C)*var2, (B-C)'*(D-C)*var2];  % [s=0, s=1, t=0, t=1]
        for i = 1:4
            if opt(i) <= 0
                if i == 1 || i == 3         % s = 0, t = 0
                    P1 = C; P2 = A;
                elseif i == 2           	% s = 1, t = 0
                    P1 = D; P2 = A;
                else                      	% t = 1, s = 0
                    P1 = C; P2 = B;
                end
            elseif opt(i) >= 1                             
                if i == 2 || i == 4         % s = 1, t = 1
                    P1 = D; P2 = B;
                elseif i == 1             	% s = 0, t = 1
                    P1 = C; P2 = B;
                else                      	% t = 0, s = 1
                    P1 = D; P2 = A;
                end
            else
                if i == 1                	% s = 0, t € [0,1]
                    P1 = C; P2 = A+opt(i)*(B-A);
                elseif i == 2              	% s = 1, t € [0,1]
                    P1 = D; P2 = A+opt(i)*(B-A);
                elseif i == 3              	% t = 0, s € [0,1]
                    P1 = C+opt(i)*(D-C); P2 = A;
                else                       	% t = 1, s € [0,1]
                    P1 = C+opt(i)*(D-C); P2 = B;                              
                end
            end
            
            d_c_temp = norm(P1-P2);
            if d_c_temp < d_c
                d_c = d_c_temp;
                plane = [P1; P2-P1];
            end
        end
    end
end
