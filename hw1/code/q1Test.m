function q1Test(A)

% from self written code
[L, D, U, P] = q1(A);
fprintf('Printing P*A');
P*A
fprintf('Priting L*D*U');
L*D*U

end