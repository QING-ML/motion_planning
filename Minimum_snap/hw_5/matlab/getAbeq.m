function [Aeq, beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    %
    %
    %
    %
    
    for k = 0: size(start_cond,2)- 1
        for i = k:n_order
            Aeq_start(k+1,i+1) = factorial(i)/factorial(i - k) * 0^(i-k) ;   
        end
    end
    
    beq_start(1) = start_cond(1);
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %
    %
    %
    %
    
    for k = 0:size(end_cond,2) - 1
        for i = k:n_order
            Aeq_end(k+1,n_all_poly - (7-i)) = factorial(i)/factorial(i - k) * ts(n_seg) ^ (i - k) ;   
        end
    end
    
    beq_end(1) = end_cond(1);
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    %
    %
    %
    %
    for row = 1:n_seg - 1
        for col = (row - 1) * (n_order + 1) + 1 : (row - 1) * (n_order + 1) + 8
            k = 0;
            i = col - ((row - 1) * (n_order + 1) + 1);
            Aeq_wp(row, col) = factorial(i)/factorial(i - k) * ts(row) ^ (i - k);
        end
    end
    
    for row = 1:n_seg - 1
        beq_wp(row) = waypoints(row + 1);
    end
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %
    %
    %
    %
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
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    %
    %
    %
    %
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
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    %
    %
    %
    %
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
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    %
    %
    %
    %
    k = 3;
    for row = 1:n_seg-1
        for col = (row - 1) * (n_order + 1) + k + 1 : (row - 1) * (n_order + 1) + 8
             i = col - ((row - 1) * (n_order + 1) + 1);
            Aeq_con_j(row, col) = factorial(i)/factorial(i - k) * ts(row) ^ (i - k);
        end
        for col = (row - 1) * (n_order + 1) + k + 9 : (row - 1) * (n_order + 1) + 16
             i = col - ((row - 1) * (n_order + 1) + 9);
             Aeq_con_j(row, col) = - ( factorial(i)/factorial(i - k) * 0 ^ (i - k));
        end
    end    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end