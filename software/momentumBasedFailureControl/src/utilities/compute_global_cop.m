function [global_cop, robotOnGround] = compute_global_cop(wrench_left,wrench_right, H_left, H_right)

p_left_foot  = H_left(1:3, 4);
p_right_foot = H_right(1:3, 4);

global_cop    = zeros(3,1);
threshold     = 5;
eps           = 0.0;
robotOnGround = 0;

if (abs(wrench_left(3)) > threshold) && (abs(wrench_right(3)) > threshold)

    robotOnGround = 1;
    cop_left      = [[-wrench_left(5); wrench_left(4)] / (wrench_left(3) + eps); 0.0] + [p_left_foot(1:2); 0.0];
    cop_right     = [[-wrench_right(5); wrench_right(4)] / (wrench_right(3) + eps); 0.0] + [p_right_foot(1:2); 0.0];
    global_cop    = (cop_left * wrench_left(3) + cop_right * wrench_right(3))/(wrench_left(3) + wrench_right(3) + eps);
end
end
