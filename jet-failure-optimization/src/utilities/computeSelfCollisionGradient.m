function gradient = computeSelfCollisionGradient(u, w_H_i, w_H_j, c_i, c_j, b_c_i, b_c_j, J_i, J_j, Config)

    % COMPUTESELFCOLLISIONGRADIENT compute the gradient of self collision
    %                              constraint between two spheres.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %

    w_R_i     = w_H_i(1:3,1:3);
    w_R_j     = w_H_j(1:3,1:3);

    % from angular velocity to Euler angles (rpy)
    % see also http://www.diag.uniroma1.it/~deluca/rob1_en/11_DifferentialKinematics.pdf
    roll  = u(5);
    yaw   = u(6);
    E_rpy = [ cos(roll)*cos(yaw)  -sin(yaw)   0
              cos(roll)*sin(yaw)   cos(yaw)   0
             -sin(roll)            0          1];

    % compute analytical jacobians
    Ja_i        = [J_i, zeros(6, Config.turbinesData.njets)];
    Ja_j        = [J_j, zeros(6, Config.turbinesData.njets)];
    Ja_i(:,4:6) = Ja_i(:,4:6)*E_rpy;
    Ja_j(:,4:6) = Ja_j(:,4:6)*E_rpy;

    % compute sphere centers Jacobians
    Jc_i      = Ja_i(1:3,:) - wbc.skew(w_R_i*b_c_i)*Ja_i(4:6,:);
    Jc_j      = Ja_j(1:3,:) - wbc.skew(w_R_j*b_c_j)*Ja_j(4:6,:);

    gradient  = 2*(c_i-c_j)'*(Jc_i-Jc_j);
end