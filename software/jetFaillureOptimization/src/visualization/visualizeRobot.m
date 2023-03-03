function [] = visualizeRobot(u, KinDynModel, Config)

    % VISUALIZEROBOT visualize the robot, jet forces and collisions.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % set robot in the correct configuration
    iDynTreeWrappers.setRobotState(KinDynModel, wbc.fromPosRpyToTransfMatrix([u(1:3); u(4:6)]), ...
                                   u(7:end-Config.turbinesData.njets), zeros(6,1), zeros(Config.ndof,1), Config.gravityAcc);

    % visualize the robot
    robotFig = iDynTreeWrappers.prepareVisualization(KinDynModel, Config.meshesPath, 'color', [0.9,0.9,0.9], 'material', 'metal', ...
                                                     'transparency', 0.7, 'debug', true, 'view', [-92.9356 22.4635],...
                                                     'groundOn', true, 'groundColor', [0.5 0.5 0.5], 'groundTransparency', 0.5);

    % get figure number
    figNum = robotFig.mainHandler.Number;

    % visualize jet thusts
    plotJetsArrows(u, KinDynModel, Config, figNum);

    % visualize all collision spheres
    for i = 1:length(Config.collisions.framesList)
 
        w_H_i      = iDynTreeWrappers.getWorldTransform(KinDynModel, Config.collisions.framesList{i}); 
        centers_i  = computeSelfCollisionCenters(w_H_i, Config.collisions.(Config.collisions.framesList{i}).centers);
        radiuses_i = Config.collisions.(Config.collisions.framesList{i}).radiuses;

        plotSelfCollisions(centers_i, radiuses_i, figNum)
    end

    % remove faulty turbine from the list of jets
    switch Config.FAILURE_TYPE

        case 1
            activeJetsNumber = [1, 3, 4];
        case 2
            activeJetsNumber = [1, 2, 4];

        otherwise
            activeJetsNumber = [1, 2, 3, 4];
    end

    % visualize jet cones
    for i = activeJetsNumber

        w_H_j_i   = iDynTreeWrappers.getWorldTransform(KinDynModel,Config.turbinesData.turbineList{i});
        w_R_j_i   = w_H_j_i(1:3,1:3);
        w_o_j_i   = w_H_j_i(1:3,4);

        offsets_i = Config.cones.(Config.cones.framesList{i}).offsets;
        axes_i    = Config.cones.(Config.cones.framesList{i}).axes;

        plotJetsCones(w_R_j_i, w_o_j_i, offsets_i, axes_i, figNum)
    end
end