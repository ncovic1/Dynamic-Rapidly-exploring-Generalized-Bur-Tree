function [xyz, nsa] = DirectKinematics(robot, q)
    xyz = zeros(3,robot.N_links+1);

    if robot.dim == 2     % Planar arm
        q1 = 0;
        for k = 1:robot.N_links
            xyz(1,k+1) = xyz(1,k) + robot.DH_table(k,2)*cos(q(k)+q1);
            xyz(2,k+1) = xyz(2,k) + robot.DH_table(k,2)*sin(q(k)+q1);
            q1 = q1 + q(k);
        end      
        nsa = [];
        
    else    % 3D Arm
        Tc = eye(4);
        ii = 2;
        for i = 1:robot.N_DOF
            Tc = Tc * GetDH_matrix(q(i)+robot.offsets(i), robot.DH_table(i,1), robot.DH_table(i,2), robot.DH_table(i,3));
            if ~prod(Tc(1:3,4) == xyz(:,ii-1))
                xyz(:,ii) = Tc(1:3,4);
                ii = ii + 1;
            end
        end
        nsa = Tc(1:3,1:3);
    end
end

function DH = GetDH_matrix(theta, d, a, alpha)
    % output: DH matrix
    % input: DH parameters:
    % theta: rotation along z(n) axis
    % d: traslation along z(n) axis
    % a: translation along x(n+1) axis
    % alpha: rotation along x(n+1) axis
    % all angles are expressed in degrees

    DH = [cos(theta)   -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);
          sin(theta)   cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta);
          0            sin(alpha)              cos(alpha)              d;
          0            0                       0                       1];

end