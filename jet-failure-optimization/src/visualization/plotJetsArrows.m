function [] = plotJetsArrows(u, KinDynModel, Config, figNum)

    % PLOTJETSARROWS visualize jets intensities on the robot.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % turbines and joints data
    njets           = Config.turbinesData.njets;
    turbineList     = Config.turbinesData.turbineList;
    turbineAxis     = Config.turbinesData.turbineAxis;
    jetsIntMax      = max(Config.opti.upperBound(end-njets+1:end));
    jetsIntensities = u(end-njets+1:end);

    % iterate on njets to calculate jets quantities
    for i = 1:njets

        % forward kinematics
        w_H_j_i      = iDynTreeWrappers.getWorldTransform(KinDynModel,turbineList{i});
        w_R_j_i      = w_H_j_i(1:3,1:3);
        w_o_j_i      = w_H_j_i(1:3,4);
        l_jet_i      = sign(turbineAxis(i))*w_R_j_i(1:3,abs(turbineAxis(i)));
        w_end_jets_i = w_o_j_i - l_jet_i*0.5*jetsIntensities(i)/jetsIntMax;
    
        figure(figNum)
        hold on
        plot3([w_o_j_i(1) w_end_jets_i(1)],[w_o_j_i(2) w_end_jets_i(2)],[w_o_j_i(3) w_end_jets_i(3)],'r','linewidth',8)
    end
end