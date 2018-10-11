function [L, D, U, P] = q1(A)
% PLU_DECOMPOSE Summary of function goes here
% A should be a n*n matrix
% P is the permutation matrix that comes from pivoting
% L is a lower triangular matrix
% U is an upper triangular matrix
% D is a diagonal matrix

[m, n] = size(A);
if m ~= n
    fprintf('A is not a square matrix.\n');
    return;
end

% pivoting cannot be performed
% NaN values are returned - ensuring that rank = n
for i = 1:n
    b = A(:,i);
    if 1 && ~any(b')
        fprintf('Aborting have zero column in matrix\n');
        return;
    end
end

L = eye(n);
P = L;
U = A;

for k = 1:n
    [~, m] = max(abs(U(k:n, k)));
    m = m+k-1;
    
    % check if the pivot for the kth operation is the kth diagonal element
    % else the pivot is in the mth row
    if m ~= k
        % interchange rows m and k in U
        temp = U(k, :);
        U(k, :) = U(m, :);
        U(m, :) = temp;
        
        % interchange rows m and k in P
        temp = P(k, :);
        P(k, :) = P(m, :);
        P(m, :) = temp;
        
        % interchange rows m and k but only with the first k-1 columns of L
        if k >= 2
            temp = L(k, 1:k-1);
            L(k, 1:k-1) = L(m, 1:k-1);
            L(m, 1:k-1) = temp;
        end
    end
    
    for j = k+1:n
        % pivot the elements
        L(j, k) = U(j, k) / U(k, k);
        % find the columns of U
        U(j, :) = U(j, :) - L(j, k)*U(k, :);
    end
end


% get D from U
D = diag(diag(U));
diagvec = diag(D);
for l = 1:n
    if(diagvec(l) ~= 0)
        U(l,:) = U(l,:) / diagvec(l);
    else
        U(l,:) = U(l,:);
        U(l,l) = 1;
    end
end
end