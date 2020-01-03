function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = [];
    beq_start = [];
    Aeq_start = zeros(3, n_all_poly);
    beq_start = zeros(3, 1);

    for k = 0: size(start_cond,2)- 1
        for i = k:n_order
            Aeq_start(k+1,i+1) = factorial(i)/factorial(i - k) * 0^(i-k) ;   
        end
    end
    
    beq_start(1) = start_cond(1);    
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = [];
    beq_end = [];
    Aeq_end = zeros(3, n_all_poly);
    beq_end = zeros(3, 1);
    
    for k = 0:size(end_cond,2) - 1
        for i = k:n_order
            Aeq_end(k+1,n_all_poly - (7-i)) = factorial(i)/factorial(i - k) * ts(n_seg) ^ (i - k) ;   
        end
    end
    
    beq_end(1) = end_cond(1);    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = [];
    beq_con_p = [];
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    
    k = 0;
    for row = 1:n_seg-1
        for col = (row - 1) * (n_order + 1) + k + 1 : (row - 1) * (n_order + 1) + 8
             i = col - ((row - 1) * (n_order + 1) + 1);
            Aeq_con_p(row, col) = factorial(i)/factorial(i - k) * ts(row) ^ (i - k);
        end
        for col = (row - 1) * (n_order + 1) + k + 9 : (row - 1) * (n_order + 1) + 16
             i = col - ((row - 1) * (n_order + 1) + 9);
             Aeq_con_p(row, col) = - ( factorial(i)/factorial(i - k) * 0 ^ (i - k));
        end
    end    
    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = [];
    beq_con_v = [];
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    
    k = 1;
    for row = 1:n_seg-1
        for col = (row - 1) * (n_order + 1) + k + 1 : (row - 1) * (n_order + 1) + 8
             i = col - ((row - 1) * (n_order + 1) + 1);
            Aeq_con_v(row, col) = factorial(i)/factorial(i - k) * ts(row) ^ (i - k);
        end
        for col = (row - 1) * (n_order + 1) + k + 9 : (row - 1) * (n_order + 1) + 16
             i = col - ((row - 1) * (n_order + 1) + 9);
             Aeq_con_v(row, col) = - ( factorial(i)/factorial(i - k) * 0 ^ (i - k));
        end
    end    
    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = [];
    beq_con_a = [];
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    
    k = 2;
    for row = 1:n_seg-1
        for col = (row - 1) * (n_order + 1) + k + 1 : (row - 1) * (n_order + 1) + 8
             i = col - ((row - 1) * (n_order + 1) + 1);
            Aeq_con_a(row, col) = factorial(i)/factorial(i - k) * ts(row) ^ (i - k);
        end
        for col = (row - 1) * (n_order + 1) + k + 9 : (row - 1) * (n_order + 1) + 16
             i = col - ((row - 1) * (n_order + 1) + 9);
             Aeq_con_a(row, col) = - ( factorial(i)/factorial(i - k) * 0 ^ (i - k));
        end
    end    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end