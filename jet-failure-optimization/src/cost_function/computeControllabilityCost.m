function contrCost = computeControllabilityCost(u, KinDynModel, Config)
      
    % COMPUTECONTROLLABILITYCOST calculates a function that must be
    %                            minimized in order to maximize
    %                            manipulability of the i/o matrix.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % get the momentum gradient
    [~, grad_LDot] = computeMomentumDerivativeAndGradient(u, KinDynModel, Config);

    % get i/o matrix and controllability cost
    tol         = 1e-5;
    contrMatrix = grad_LDot(:,7:Config.ndof+6-12);
    contrCost   = 1/(sqrt(det((contrMatrix)*transpose(contrMatrix))) + tol);
end