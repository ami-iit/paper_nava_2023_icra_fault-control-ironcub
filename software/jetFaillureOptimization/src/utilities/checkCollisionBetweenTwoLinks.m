function [c_coll_ij, grad_coll_ij, isColliding] = checkCollisionBetweenTwoLinks(u, w_H_i, w_H_j, centers_i, centers_j, b_centers_i, b_centers_j, radiuses_i, radiuses_j, J_i, J_j, Config)

    % CHECKCOLLISIONSBETWEENTWOLINKS compute the collision constraints 
    %                                between and two links and the 
    %                                associated gradients. 
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %
    c_coll_ij    = [];
    grad_coll_ij = [];
    scaling      = 1;
    isColliding  = false;

    % iterate on the collision elements
    for i = 1:size(centers_i, 1)

        b_c_i = b_centers_i(i,:)';
        c_i   = centers_i(i,:)'; 
        r_i   = radiuses_i(i);

        for j = 1:size(centers_j, 1)
     
            b_c_j = b_centers_j(j,:)';
            c_j   = centers_j(j,:)';
            r_j   = radiuses_j(j);

            % condition for collision avoidance: |c_i - c_j|^2 > (r_i + r_j)^2

            if ((c_i-c_j)'*(c_i-c_j) <= scaling*(r_i + r_j)^2)

                isColliding = true;
            end

            c_coll_ij = [c_coll_ij; (-(c_i-c_j)'*(c_i-c_j) +scaling*(r_i + r_j)^2)]; %#ok<AGROW> 

            % compute the corresponding gradient
            grad_coll_ij = [grad_coll_ij; -computeSelfCollisionGradient(u, w_H_i, w_H_j, c_i, c_j, b_c_i, b_c_j, J_i, J_j, Config)]; %#ok<AGROW> 
        end
    end
end