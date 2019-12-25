function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = [];
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        %
        %
        %
        %
        M_k = zeros(n_order+1,n_order+1);

        for derivative = 0:3
            for col = derivative+1:n_order+1
                i = col - 1;
                M_k(1+derivative,col) =  factorial(i)/factorial(i - derivative) * 0 ^ (i - derivative);
            end
        end
        
        for derivative = 0:3
            for col = derivative+1:n_order+1
                i = col - 1;
                M_k(5+derivative,col) =  factorial(i)/factorial(i - derivative) * ts(k) ^ (i - derivative);
            end
        end
                
        M = blkdiag(M, M_k);
    end
end