%%main
step_n= 1500;
conical_step = step_n + 20
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
[Target_px, Target_py, Target_pz] = generate_conicalspiral(conical_step);
log_x = getTrajectory(px_0, up_v_xy, bott_v_xy, up_a_xy, bott_a_xy, up_j_xy, bott_j_xy, Target_px, step_n);
log_y = getTrajectory(py_0, up_v_xy, bott_v_xy, up_a_xy, bott_a_xy, up_j_xy, bott_j_xy, Target_py, step_n);
log_z = getTrajectory(pz_0, up_v_z, bott_v_z, up_a_z, bott_a_z, up_j_z, bott_j_z, Target_pz, step_n);

result_px = log_x(:, 2);
result_py = log_y(:, 2);
result_pz = log_z(:, 2);

plot3(result_px, result_py, result_pz);