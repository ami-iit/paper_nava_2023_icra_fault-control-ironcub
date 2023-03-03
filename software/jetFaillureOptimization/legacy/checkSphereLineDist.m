while true

    clc
    close all
    clear

    rng('shuffle')

    % generate a random line
    w_p_0  = 5*rand(3,1);
    w_a_0  = 2*rand(3,1);

    % normalize axis
    w_a_0  = w_a_0/norm(w_a_0);

    % parametrized line
    line_p = @(t) w_p_0 + w_a_0*t;

    % generate a random sphere
    w_c    = 3*rand(3,1);
    R      = 2*abs(rand);

    %---------------------------------------------------------------------%

    % plot sphere and line
    [x,y,z]   = sphere(10);
    x         = x*R;
    y         = y*R;
    z         = z*R;
    line_plot = [line_p(-3), line_p(0), line_p(3)];

    figure
    surf(x+w_c(1), y+w_c(2), z+w_c(3))
    axis equal
    hold on
    grid on
    line(line_plot(1,:),line_plot(2,:),line_plot(3,:))

    %---------------------------------------------------------------------%
    
    % check distance calculation
    N_a               = eye(3) - w_a_0*w_a_0';
    my_distance_sq    = (w_p_0-w_c)'*N_a*(w_p_0-w_c);
    my_check_dist_sq  = my_distance_sq <= R^2;

    % groundthruth
    t_min_dist               = w_a_0(1)*(w_c(1)-w_p_0(1)) + w_a_0(2)*(w_c(2)-w_p_0(2)) + w_a_0(3)*(w_c(3)-w_p_0(3));
    line_p_min_dist          = line_p(t_min_dist); 
    groundthr_distance_check = norm(line_p_min_dist-w_c) <= R;

    disp('Min distance [my check, groundthruth]:')
    disp(num2str([(sqrt(my_distance_sq)-R), (norm(line_p_min_dist-w_c)-R)]))

    disp('is colliding [my check, groundthruth]:')
    disp(num2str([my_check_dist_sq, groundthr_distance_check]))

    pause
end
