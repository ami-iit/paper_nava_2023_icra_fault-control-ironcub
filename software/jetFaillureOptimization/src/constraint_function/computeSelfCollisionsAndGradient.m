function [c_collision, grad_c_collision] = computeSelfCollisionsAndGradient(u, KinDynModel, Config)

    % COMPUTESELFCOLLISIONANDGRADIENT compute the robot self collision
    %                                 constraints and the associated
    %                                 gradients.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %

    c_collision      = [];
    grad_c_collision = [];

    % iterate on the list of links with collisions
    for i = 1:length(Config.collisions.framesList)

        frame_i     = Config.collisions.framesList{i};
        J_i         = iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel, frame_i);
        w_H_i       = iDynTreeWrappers.getWorldTransform(KinDynModel, frame_i);
        b_centers_i = Config.collisions.(frame_i).centers;
        centers_i   = computeSelfCollisionCenters(w_H_i, b_centers_i);
        radiuses_i  = Config.collisions.(frame_i).radiuses;

        for j = 1:length(Config.collisions.framesList)

            % if the collision algorithm is currently checking the collision
            % between chest and left/right jetpack turbines, skip the check.
            if (i > j) && ~((i == 12 && j == 3) || (i == 12 && j == 4))

                frame_j     = Config.collisions.framesList{j};
                J_j         = iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel, frame_j);
                w_H_j       = iDynTreeWrappers.getWorldTransform(KinDynModel, frame_j);
                b_centers_j = Config.collisions.(frame_j).centers;
                centers_j   = computeSelfCollisionCenters(w_H_j, b_centers_j);
                radiuses_j  = Config.collisions.(frame_j).radiuses;

                [c_coll_ij, grad_coll_ij, isColliding] = checkCollisionBetweenTwoLinks(u, w_H_i, w_H_j, centers_i, centers_j, b_centers_i, b_centers_j, radiuses_i, radiuses_j, J_i, J_j, Config);

                if isColliding && Config.DEBUG_COLLISIONS

                    disp(['Collision detected: link ', frame_i, ' and link ', frame_j])
                end

                c_collision      = [c_collision; c_coll_ij]; %#ok<AGROW>
                grad_c_collision = [grad_c_collision; grad_coll_ij]; %#ok<AGROW>
            end
        end
    end
end