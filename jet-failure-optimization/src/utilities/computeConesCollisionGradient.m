function gradient = computeConesCollisionGradient(u, w_R_i, w_R_j, c_i, o_j, p_j, b_c_i, a_j, J_i, J_j, Config)

    % COMPUTECONESCOLLISIONGRADIENT compute the gradient of jet-cones 
    %                               collision constraints.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %

    N_a = eye(3) - a_j*a_j';
    
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
    Jp_j      = Ja_j(1:3,:) - wbc.skew(w_R_j*o_j)*Ja_j(4:6,:);

    gradient  = 2*(p_j-c_i)'*N_a*(Jp_j-Jc_i) -2*(p_j-c_i)'*(a_j*a_j')*wbc.skew((p_j-c_i))*Ja_j(4:6,:);
end