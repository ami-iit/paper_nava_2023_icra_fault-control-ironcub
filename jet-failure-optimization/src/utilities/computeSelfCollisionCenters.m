function centers_w = computeSelfCollisionCenters(w_H_i, centers_b)

    % COMPUTESELFCOLLISIONCENTERS compute the center of links' self-collisions 
    %                             in the inertial reference frame.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %

    % get frame rotation and origin
    w_R_i     = w_H_i(1:3,1:3);
    w_o_i     = w_H_i(1:3,4);

    centers_w = zeros(size(centers_b));

    % get centers in intertial coordinates
    for k = 1:size(centers_b, 1)
    
        centers_w(k,:) = w_o_i + w_R_i*centers_b(k,:)';
    end
end