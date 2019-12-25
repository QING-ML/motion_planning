function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    %
    %
    %
    %
    %
    Ct = zeros(8*n_seg, 4*n_seg + 4);
    for n= 1:n_seg
         if n == 1
             for row = 1:4
                 col =row;
                 Ct(row, col) = 1;
             end
             
             for row = 5:8
                 if row == 5
                   Ct(row, row) = 1;
                 else
                     col = 7 +n_seg + row -5;
                     Ct(row, col) = 1;
                 end
             end
         
                          
         elseif n == n_seg
                 for row = 8 * (n -1) +1 : 8 * (n -1) +4
                     if row == 8 * (n -1) +1
                         col = 4 + n_seg - 1;
                         Ct(row, col) = 1;
                     else
                         col = 7 + n_seg + 3 * (n -2) + row - 8 * (n -1) - 1;
                         Ct(row, col) = 1;
                     end
                 end
                 i =1;
                 for row = 8 * (n -1) +5 : 8 * (n -1) +8
                     col = 4 + n_seg - 1 + i;
                     Ct(row, col) = 1;
                     i = i + 1;
                 end
         else
             for row = (n-1)*8 + 1:(n-1)*8 + 8
                 if row == (n-1)*8 + 1
                     col = 4 + n -1;
                     Ct(row, col) = 1;
                 elseif row == (n-1)*8 + 5
                     col = 4 +n;
                     Ct(row, col) = 1;
                 else
                     if row -(n -1)*8 -1 <= 3
                         col = 7 + n_seg + 3 * (n -2)  +  row -(n -1)*8 -1;
                         Ct(row, col) = 1;
                     else
                         col = 7 + n_seg + 3 * (n -1) + row -(n -1)*8 - 5;
                         Ct(row, col) = 1;
                     end
                 end
             end
         end
    end

        
end