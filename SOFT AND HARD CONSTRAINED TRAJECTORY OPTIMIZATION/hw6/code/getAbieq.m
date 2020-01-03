function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p = [];
    bieq_p = [];
    Aieq_p = zeros(2*n_all_poly, n_all_poly);
    

    %% < range_max
    col = 1;
    for row = 1:n_all_poly
        Aieq_p(row, col) = 1;
        col = col + 1;
    end
    %% > range_min
    col = 1;
    for row = n_all_poly + 1: 2*n_all_poly
        Aieq_p(row, col) = -1;
        col = col + 1;
    end
    
    %%assignment bieq
    for i = 1:n_seg
            for j = 1:n_order+1
                bieq_p((i-1)*(n_order+1) + j,1) =  corridor_range(i, 2);
            end

    end
    
     for i = 1:n_seg
            for j = 1:n_order+1
                bieq_p(n_all_poly +(i-1)*(n_order+1) + j,1) = - corridor_range(i, 1);
            end

    end
    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = [];
    bieq_v = [];
    Aieq_v = zeros(n_seg*(n_order),n_all_poly);
    
    col = 1;
    for row = 1:n_seg*(n_order)
        Aieq_v(row, col) = -1;
        Aieq_v(row, col+1) = 1;
        col = col + 1;
        if mod(row, n_order) == 0
            col =col + 1;
        end
    end
    
%     col = 1;
%     for row = n_seg*(n_order) + 1: 2*n_seg*(n_order)
%         Aieq_p(row, col) = 1;
%         Aieq_p(row, col+1) = -1;
%         col = col + 1;
%         if mod(row, n_order) == 0
%             col =col + 1;
%         end      
%     end
    
    %%assignment bieq_v
    for row = 1:n_seg*(n_order)
        bieq_v(row,1) = v_max/n_order;
    end
    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = [];
    bieq_a = [];
    Aieq_a = zeros(n_seg*(n_order-1),n_all_poly);
    
    col = 1;
    for row = 1:n_seg*(n_order-1)
        Aieq_a(row, col) = 1;
        Aieq_a(row, col+1) = -2;
        Aieq_a(row, col+2) = 1;
        col = col + 1;
        if mod(row, n_order-1) == 0
            col =col + 2;
        end
    end
    
    for row = 1:n_seg*(n_order-1)
        bieq_a(row,1) = a_max/(n_order*(n_order-1));
        
    end    
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    %Aieq = Aieq_p;
    %bieq = bieq_p;
end