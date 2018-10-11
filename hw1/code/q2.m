function [L,D,U,P,U1,S1,V1] = q2(A)

[L,U,P] = lu(A);
[~,n] = size(A);

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

[U1,S1,V1] = svd(A);

end