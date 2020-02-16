function [log] = getTrajectory(p_0, up_v, bott_v, up_a, bott_a, up_j, bott_j, target_p)
v_0 = 0;
a_0 = 0;
K = 20; %20 because w = 0.02 rad/s
dt = 0.2;
log = [0 p_0 v_0 a_0];
w1 = 1;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 2;

for t = 0.2:0.2:300
    %% Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K, dt, p_0, v_0, a_0);
    
    %% Target_ p - Tp should be 0
    
    
    %%Construct the optimization problem
    H = blkdiag(w4*eye(K) + w1*(Tp' * Tp) +w2*(Tv' * Tv) + w3*(Ta' * Ta), w5*eye(K));
    F = [2 * (w1 * Bp' * Tp + w2 * Bv' * Tv + w3 * Ba' *Ta), zeros(1, K)];
    
    A = [ Tv -eye(K); -Tv -eye(K); Ta -eye(K); -Ta -eye(K); eye(K) -eye(K); -eye(K) -eye(K); zeros(size(Ta)) -eye(K)];
    b = [ ones(20,1) * up_v - Bv;  ones(20,1) * bott_v + Bv; ones(20,1) * up_a - Ba; ones(20, 1) * bott_a + Ba; ones(20, 1) * up_j; ones(20.1) * bott_j; zeros(K,1)];
    
    %% Solve the optimization problem
    J = quadprog(H, F, A, b);
    
    %% Solve the optimization problem
    j = J(1);
    p_0 = p_0 + v_0 * dt + 0.5 * a_0 * dt ^ 2 + 1/6 * j * dt ^ 3;
    v_0 = v_0 + a_0 * dt + 0.5 * j * dt ^ 2;
    a_0 = a_0 + j * dt;
    
    %% Log the states
    log = [log; t p_0 v_0 a_0];
end