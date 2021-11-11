function collision = CheckCollisionBinary(q1, q2)
% Checks collision binary between q1 and q2
    
    q = 0.5*(q1+q2);
    [d_c, planes] = GetDistance(q);
    % Generate spines from q towards q1 and q2
    if d_c > 0
        collision = false;
        [q_new1, reached] = GenerateGSpine(q, q1, d_c, planes);
        if ~reached
            collision = CheckCollisionBinary(q1, q_new1);
            if collision
                return;
            end
        end
        [q_new2, reached] = GenerateGSpine(q, q2, d_c, planes);
        if ~reached
            collision = CheckCollisionBinary(q2, q_new2);
            if collision
                return;
            end
        end
    else
        collision = true;
    end
    
end


function [q_new, reached] = GenerateGSpine(q, q_e, d_c, planes)
    A = RGBT();
    q_new = q;
    d_c_temp = d_c;
    for j = 1:A.N_layers
        [q_new, reached] = A.GenerateSpine(q_new, q_e, d_c_temp);
        d_c_temp = A.Update_d_c(q_new, planes);
        if reached || norm(q_new-q_e) < A.eps/10
            reached = true;
            return;
        end
    end        
end