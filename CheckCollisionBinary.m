function collision = CheckCollisionBinary(robot, obstacles, q1, q2)
% Checks collision binary between q1 and q2

    q = 0.5*(q1+q2);
    [collision, d_c, separatrix] = CheckCollision(robot, obstacles, q);
    % Generate spines from q towards q1 and q2
    if ~collision
        [q_new1, reached] = GenerateGSpine(q, q1, d_c, separatrix);
        if ~reached
            collision = CheckCollisionBinary(robot, obstacles, q1, q_new1);
            if collision
                return;
            end
        end
        [q_new2, reached] = GenerateGSpine(q, q2, d_c, separatrix);
        if ~reached
            collision = CheckCollisionBinary(robot, obstacles, q2, q_new2);
            if collision
                return;
            end
        end
    end
    
end


function [q_new, reached] = GenerateGSpine(q, q_e, d_c, separatrix)
    A = RGBT();
    q_new = q;
    d_c_temp = d_c;
    for j = 1:A.N_layers
        [q_new, reached] = A.GenerateSpine(q_new, q_e, d_c_temp);
        d_c_temp = A.Update_d_c(q_new, separatrix);
        if reached || norm(q_new-q_e) < A.eps0
            reached = true;
            return;
        end
    end        
end