function [c_collCones_ij, grad_collCones_ij, isColliding] = checkCollisionBetweenLinksAndCones(u, centers_i, b_centers_i, radiuses_i, w_R_i, J_i, ...
                                                                                               offsets_j, axes_j, w_R_j, w_o_j, J_j, Config)
    % CHECKCOLLISIONSBETWEENLINKSANDCONES compute the collision constraints 
    %                                     between and a link (approx. with
    %                                     spheres) and the jets cones (approx.
    %                                     with lines), and the associated 
    %                                     gradients. 
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %

    c_collCones_ij    = [];
    grad_collCones_ij = [];
    scaling           = 1;
    isColliding       = false;

    % iterate on the link collision elements
    for i = 1:size(centers_i, 1)

        b_c_i = b_centers_i(i,:)';
        c_i   = centers_i(i,:)'; 
        r_i   = radiuses_i(i);

        % iterate on the jet cone collision elements
        for j = 1:size(offsets_j, 1)
     
            p_j = w_o_j + w_R_j*offsets_j(j,:)';
            a_j = w_R_j*axes_j(j,:)';
            N_a = eye(3) - a_j*a_j';

            % condition for collision avoidance: (p_j - c_i)'*N_a*(p_j - c_i) > r_i^2
            if ((p_j - c_i)'*N_a*(p_j - c_i) <= scaling*(r_i^2))

                isColliding = true;
            end

            c_collCones_ij = [c_collCones_ij; (-(p_j - c_i)'*N_a*(p_j - c_i) +scaling*(r_i)^2)]; %#ok<AGROW> 

            % compute the corresponding gradient
            grad_collCones_ij = [grad_collCones_ij; -computeConesCollisionGradient(u, w_R_i, w_R_j, c_i, offsets_j(j,:)', p_j, b_c_i, a_j, J_i, J_j, Config)]; %#ok<AGROW> 
        end
    end
end