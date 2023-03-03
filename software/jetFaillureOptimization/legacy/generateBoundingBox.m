function [v_coord, v_idx, w_center] = generateBoundingBox(w_H_j, boxVertices)

    % GENERATEBOUNDINGBOX list vertices and connections of the bounding box 
    %                     for the specified robot link.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % get frame rotation and origin
    w_R_j      = w_H_j(1:3,1:3);
    w_o_j      = w_H_j(1:3,4);

    % construct vertex coordinates
    w_v_1        = w_o_j + w_R_j*boxVertices(1,:)';
    w_v_2        = w_o_j + w_R_j*boxVertices(2,:)';
    w_v_3        = w_o_j + w_R_j*boxVertices(3,:)';
    w_v_4        = w_o_j + w_R_j*boxVertices(4,:)';
    w_v_5        = w_o_j + w_R_j*boxVertices(5,:)';
    w_v_6        = w_o_j + w_R_j*boxVertices(6,:)';
    w_v_7        = w_o_j + w_R_j*boxVertices(7,:)';
    w_v_8        = w_o_j + w_R_j*boxVertices(8,:)';

    v_coord  = [w_v_1'; w_v_2'; w_v_3'; w_v_4'; w_v_5'; w_v_6'; w_v_7'; w_v_8'];
    v_idx    = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';

    % get the center of the box
    w_center = (w_v_8 + w_v_2)/2;
end