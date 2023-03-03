function plotBoundingBox(w_H_j, boxVertices, figNum)

    % PLOTBOUNDINGBOX plots the bounding box for the specified robot link.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    [v_coord, v_idx, w_center] = generateBoundingBox(w_H_j, boxVertices);

    % isolate x,y,z coordinates
    v_x = v_coord(:,1);
    v_y = v_coord(:,2);
    v_z = v_coord(:,3);

    % plot the box
    figure(figNum)
    hold on
    patch(v_x(v_idx), v_y(v_idx), v_z(v_idx), 'r', 'facealpha', 0.1);
    view(3);

    % plot the center of the box
    plot3(w_center(1), w_center(2), w_center(3), '.k', 'markersize', 20)
end