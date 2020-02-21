%%main
horizontal_n = 30;
conical_step = 1500;
step_n= horizontal_n + conical_step;
%start position
px_0 = 8;
py_0 = 0;
pz_0 = 20;
%% all threshold should be positiv
%limit v
up_v_xy = 6;
bott_v_xy = 6;
up_v_z = 6;
bott_v_z = 1;
%limit_a
up_a_xy = 3;
bott_a_xy = 3;
up_a_z = 3;
bott_a_z = 1;
%limit_j
up_j_xy = 3;
bott_j_xy = 3;
up_j_z = 2;
bott_j_z =2; 

%% main function
[Target_px, Target_py, Target_pz] = generate_conicalspiral(conical_step, horizontal_n);
log_x = getTrajectory(px_0, up_v_xy, bott_v_xy, up_a_xy, bott_a_xy, up_j_xy, bott_j_xy, Target_px, step_n);
log_y = getTrajectory(py_0, up_v_xy, bott_v_xy, up_a_xy, bott_a_xy, up_j_xy, bott_j_xy, Target_py, step_n);
log_z = getTrajectory(pz_0, up_v_z, bott_v_z, up_a_z, bott_a_z, up_j_z, bott_j_z, Target_pz, step_n);

%%plot
result_px = log_x(:, 2);
result_py = log_y(:, 2);
result_pz = log_z(:, 2);

px_v = log_x(:, 3);
py_v = log_y(:, 3);
pz_v = log_z(:, 3);

px_a = log_x(:, 4);
py_a = log_y(:, 4);
pz_a = log_z(:, 4);

px_j = log_x(:, 5);
py_j = log_y(:, 5);
pz_j = log_z(:, 5);

t = log_x(:, 1);

figure(1)
p1 = plot3(result_px, result_py, result_pz,'--', 'linewidth', 1);
hold on
p2 = plot3(Target_px, Target_py, Target_pz);
hold off
title('Trajectory')
legend([p1, p2],{'real trajectory', 'reference trajectory'})

figure(2)
plot(t, px_v, t, py_v,'--' , t, pz_v, ':');
title('Velocity')
legend({'y = px_v','y = py_v', 'y = pz_v'})

figure(3)
plot(t, px_a, t, py_a, t, pz_a);
title('Acceleration')
legend({'y = px_a','y = py_a', 'y = pz_a'})

figure(4)
plot(t, px_j, t, py_j, t, pz_j);
title('Jerk')
legend({'y = px_j','y = py_j', 'y = pz_j'})
