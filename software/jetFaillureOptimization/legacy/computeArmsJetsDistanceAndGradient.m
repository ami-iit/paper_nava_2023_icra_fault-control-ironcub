function [armsDist, grad_armsDist] = computeArmsJetsDistanceAndGradient(~, KinDynModel, Config)

    % COMPUTEARMSJETSDISTANCES computes distance (square norm) between the
    %                          arms' turbines and a specified frame, and its gradient.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % turbines and joints data
    njets       = Config.turbinesData.njets;
    turbineList = Config.turbinesData.turbineList;
   
    % arms turbines jacobians (hard-coded: they are first two turbines in
    % the turbines list)
    J_frame_full = iDynTreeWrappers.getCenterOfMassJacobian(KinDynModel);
    J_frame      = J_frame_full(1:3,:);
    J_j_leftArm  = iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel,turbineList{1});
    J_j_rightArm = iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel,turbineList{2});
    J_r_leftArm  = J_j_leftArm(1:3,:)  - J_frame;
    J_r_rightArm = J_j_rightArm(1:3,:) - J_frame;

    % forward kinematics
    w_o_frame      = iDynTreeWrappers.getCenterOfMassPosition(KinDynModel);
    w_H_j_leftArm  = iDynTreeWrappers.getWorldTransform(KinDynModel,turbineList{1});
    w_o_j_leftArm  = w_H_j_leftArm(1:3,4);
    w_H_j_rightArm = iDynTreeWrappers.getWorldTransform(KinDynModel,turbineList{2});
    w_o_j_rightArm = w_H_j_rightArm(1:3,4);

    % distances between the jets position and the frame position
    r_jet_leftArm  = w_o_j_leftArm  - w_o_frame;
    r_jet_rightArm = w_o_j_rightArm - w_o_frame;

    % arms distances and gradient
    armsDist       = [transpose(r_jet_leftArm)*r_jet_leftArm; transpose(r_jet_rightArm)*r_jet_rightArm];
    grad_armsDist  = 2*[transpose(r_jet_leftArm)*[J_r_leftArm, zeros(3,njets)]; transpose(r_jet_rightArm)*[J_r_rightArm, zeros(3,njets)]];
end
