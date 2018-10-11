function [nullspaceA] = q4(n)

I = eye(n);
u = randi(1000,n,1);
u = u/norm(u);
A = I - (u*u');

x = randi(1000,n,1);
x = x/norm(x);

% rank is always n-1
rankA = rank(A);

% since A is symmetrical => A = Q*D*Q'; and Q is an orthogonal matrix and
% also D has 1 (with multiplicity N-1) and 0 (with multiplicity 1) as
% eigenvalues
[Q,D] = eig(A);
e = eig(A);

% taking SVD decomposition for nullspace of A
[U, S, V] = svd(A);
xbar = A*x;
xbar'*u