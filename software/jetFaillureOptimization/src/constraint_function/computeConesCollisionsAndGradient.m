function [c_collisionCones, grad_c_collisionCones] = computeConesCollisionsAndGradient(u, KinDynModel, Config)

    % COMPUTECONESCOLLISIONANDGRADIENT compute the robot collision w.r.t
    %                                  jet cones constraints and the
    %                                  associated gradients.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    %

    c_collisionCones      = [];
    grad_c_collisionCones = [];

    % remove faulty turbine from the list of jets
    switch Config.FAILURE_TYPE

        case 1
            activeJetsNumber = [1, 3, 4];
        case 2
            activeJetsNumber = [1, 2, 4];

        otherwise
            activeJetsNumber = [1, 2, 3, 4];
    end

    % not all collisions need to be checked. Reduce the list of links to
    % check to legs and root link
    reducedFrameListIndeces = [7 8 9 10 11];

    % iterate on the list of links with collisions
    for i = reducedFrameListIndeces

        frame_i     = Config.collisions.framesList{i};
        J_i         = iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel, frame_i);
        w_H_i       = iDynTreeWrappers.getWorldTransform(KinDynModel, frame_i);
        b_centers_i = Config.collisions.(frame_i).centers;
        centers_i   = computeSelfCollisionCenters(w_H_i, b_centers_i);
        radiuses_i  = Config.collisions.(frame_i).radiuses;

        % iterate on the list of lines composing the cones
        for j = activeJetsNumber

            % forward kinematics
            w_H_j     = iDynTreeWrappers.getWorldTransform(KinDynModel,Config.turbinesData.turbineList{j});
            J_j       = iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel, Config.turbinesData.turbineList{j});
            w_R_j     = w_H_j(1:3,1:3);
            w_o_j     = w_H_j(1:3,4);
            offsets_j = Config.cones.(Config.cones.framesList{j}).offsets;
            axes_j    = Config.cones.(Config.cones.framesList{j}).axes;

            [c_collCones_ij, grad_collCones_ij, isColliding] = checkCollisionBetweenLinksAndCones(u, centers_i, b_centers_i, radiuses_i, w_H_i(1:3,1:3), J_i, ...
                                                                                                  offsets_j, axes_j, w_R_j, w_o_j, J_j, Config);
            if isColliding && Config.DEBUG_COLLISIONS

                disp(['Collision detected: link ', frame_i, ' and jet cone ', Config.cones.framesList{j}])
            end

            c_collisionCones      = [c_collisionCones; c_collCones_ij]; %#ok<AGROW>
            grad_c_collisionCones = [grad_c_collisionCones; grad_collCones_ij]; %#ok<AGROW>
        end
    end
end