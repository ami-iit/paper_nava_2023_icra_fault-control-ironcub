function [] = plotJetsCones(w_R_j_i, w_o_j_i, offsets_i, axes_i, figNum)

    % PLOTJETSCOMES visualize jets cones.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Sept. 2022
    %

    for k = 1:size(offsets_i, 1)

        p_k = w_o_j_i + w_R_j_i*offsets_i(k,:)';
        a_k = w_R_j_i*axes_i(k,:)';

        % line parametrization
        lx           = @(p) p_k(1) + a_k(1)*p;
        ly           = @(p) p_k(2) + a_k(2)*p;
        lz           = @(p) p_k(3) + a_k(3)*p;

        figure(figNum)
        hold on
        plot3(p_k(1), p_k(2), p_k(3), '.k', 'markersize', 20)
        line([lx(0); lx(-0.5)], [ly(0); ly(-0.5)], [lz(0); lz(-0.5)], 'linewidth', 2)
    end
end