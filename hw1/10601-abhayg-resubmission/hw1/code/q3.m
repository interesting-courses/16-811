function [xbar, result] = q3(A,b)

[~,n] = size(A);
Ab = [A b];

rankA = rank(A);
rankAb = rank(Ab);

if rankA < rankAb
    % b does not belong to column space of A => there is a least squared
    % solution given by xbar (SVD solution), no exact solution exists
    result = 0;
else
    if (rankA == n && rankAb == n)
        % b belongs to column space of A => there is a particular solution
        % since A is full rank and invertible => there is a unique solution
        result = 1;
    else
        if (rankA < n && rankAb < n && rankA == rankAb)
            % b belongs to column space of A => there is a particular solution
            % A is not full rank => not invertible => solutions to the form
            % xbar + Xn where Xn belongs to null space of A
            result = Inf;
        end
    end
end

[U, S, V] = svd(A);
xbar = V*(pinv(S)*(U'*b));

end