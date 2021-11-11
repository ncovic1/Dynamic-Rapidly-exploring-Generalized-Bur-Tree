function [xyz, nsa] = DirectKinematics(robot, q)

    if robot.dim == 2     % Planar arm
        xyz = zeros(3,robot.N_links+1);
        q1 = 0;
        for k = 1:robot.N_links
            xyz(1,k+1) = xyz(1,k) + robot.DH_table(k,2)*cos(q(k)+q1);
            xyz(2,k+1) = xyz(2,k) + robot.DH_table(k,2)*sin(q(k)+q1);
            xyz(3,k+1) = 0;
            q1 = q1 + q(k);
        end      
        nsa = [];
        
    elseif robot.dim == 3    % 3D Arm
        d = robot.DH_table(:,1);
        a = robot.DH_table(:,2);
        alpha = robot.DH_table(:,3);

        Tc = eye(4);
        xyz = zeros(3,robot.N_links+1);
        ii = 2;
        for i=1:robot.N_DOF
            Tc = Tc*DHmatrix(q(i)+robot.offsets(i), d(i), a(i), alpha(i));
            if ~prod(Tc(1:3,4) == xyz(:,ii-1))
                xyz(:,ii) = Tc(1:3,4);
                ii = ii + 1;
            end
        end
        nsa = Tc(1:3,1:3);
    end
end