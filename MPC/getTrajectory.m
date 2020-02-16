function [log] = getTrajectory(p_0, up_v, bott_v, up_a, bott_a, up_j, bott_j, Target_p, step_n)
v_0 = 0;
a_0 = 0;
j_0 = 0;
K = 20; %20 because w = 0.08 rad/s
dt = 0.2;
log = [0 p_0 v_0 a_0 j_0];
w1 = 5;
final_time = 0.2 * step_n;
%w2 = 1;
%w3 = 1;
w4 = 1;
w5 = 10;

%% diemension
% J 20*1
% T 20*20
%target_p 1 * 20
%Bp 20 * 1

%%
count = 1;
for t = 0.2:0.2:final_time
    %% Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K, dt, p_0, v_0, a_0);
    
    %% Target_ p - Tp should be 0
    % Target_p_20: further 20 step target position
    Target_p_20 = Target_p(count : count + K - 1);
    %%Construct the optimization problem
    H = blkdiag(w4*eye(K) + w1*(Tp' * Tp) , w5*eye(K));
    F = [2 * w1 * (Bp' * Tp - Target_p_20 * Tp ), zeros(1, K)];
    
    A = [ Tv -eye(K); -Tv -eye(K); Ta -eye(K); -Ta -eye(K); eye(K) -eye(K); -eye(K) -eye(K); zeros(size(Ta)) -eye(K)];
    b = [ ones(K,1) * up_v - Bv;  ones(K,1) * bott_v + Bv; ones(K,1) * up_a - Ba; ones(K, 1) * bott_a + Ba; ones(K, 1) * up_j; ones(K,1) * bott_j; zeros(K,1)];
    count = count +1;
    %% Solve the optimization problem
    J = quadprog(H, F, A, b);
    
    %% Solve the optimization problem
    j = J(1);
    p_0 = p_0 + v_0 * dt + 0.5 * a_0 * dt ^ 2 + 1/6 * j * dt ^ 3;
    v_0 = v_0 + a_0 * dt + 0.5 * j * dt ^ 2;
    a_0 = a_0 + j * dt;
    j_0 = j;
    %% Log the states
    log = [log; t p_0 v_0 a_0 j_0];
end