function [jointPos_star,jetsIntensities_star] = casadiOptimizationQP(b_mg, KinDynModel, Config)

optimization_type = 'nlp';
solver            = 'ipopt';
p_opts            = struct('expand',true);
s_opts            = struct();

casadi_optimizer = casadi.Opti(optimization_type);
casadi_optimizer.solver(solver, p_opts, s_opts);

% optimization variables
jetsIntensities  = casadi_optimizer.variable(Config.TurbinesData.njets);
jointPos         = casadi_optimizer.variable(Config.ndof);

% parameters for optimization
b_mg_cas         = casadi_optimizer.parameter(3);
jetsInt_min_cas  = casadi_optimizer.parameter(Config.TurbinesData.njets);
jetsInt_max_cas  = casadi_optimizer.parameter(Config.TurbinesData.njets);
jointPos_min_cas = casadi_optimizer.parameter(Config.ndof);
jointPos_max_cas = casadi_optimizer.parameter(Config.ndof);

% weights
weights_momentum = Config.Optimization.weight_momentum;

% initial conditions
casadi_optimizer.set_initial(jetsIntensities,Config.initConds.jetIntensities_init);
casadi_optimizer.set_initial(jointPos,Config.initConds.jointPos_init);

% objective function
casadi_optimizer.minimize(weights_momentum*sumsqr(computeAjBody(jointPos,KinDynModel,Config)*jetsIntensities + b_mg_cas));

% setting Constraint
casadi_optimizer.subject_to(jointPos <= jointPos_max_cas);
casadi_optimizer.subject_to(jointPos >= jointPos_min_cas);
casadi_optimizer.subject_to(jetsIntensities <= jetsInt_max_cas);
casadi_optimizer.subject_to(jetsIntensities >= jetsInt_min_cas);

% update casadi optimizer parameters
casadi_optimizer.set_value(b_mg_cas,b_mg);
casadi_optimizer.set_value(jetsInt_min_cas,Config.Optimization.lowerBound_turbines);
casadi_optimizer.set_value(jetsInt_max_cas,Config.Optimization.upperBound_turbines);
casadi_optimizer.set_value(jointPos_min_cas,Config.Optimization.lowerBound_joints);
casadi_optimizer.set_value(jointPos_max_cas,Config.Optimization.upperBound_joints);

% Computing the solution
try
    sol                  = casadi_optimizer.solve;
    jetsIntensities_star = full(sol.value(jetsIntensities));
    jointPos_star        = full(sol.value(jointPos));
    casadi_optimizer.set_initial(casadi_optimizer.x, sol.value(casadi_optimizer.x));

catch exception
    try
        casadi_optimizer.debug.show_infeasibilities;
        disp(exception.getReport)
    catch
        disp(exception.message)
    end
end
end