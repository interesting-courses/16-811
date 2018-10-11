function [x] = q3(f, df, x0)
% f = the function to find roots for
% df = the derivate of the function f
% x0 = the best initial approximation for starting root finding
% x = the best approximated root 

f = inline(f);
df = inline(df);
x(1) = x0 - (f(x0)/df(x0));
error(1) = abs(x(1)-x0);

% tolerance = for error estimate, stop if error greater than tolerance
% nmax = max number of iterations if tolerance never approaches a small
% epsilon
tol = 1e-4;
iter = 1e3;

k = 2;
while (error(k-1) >= tol) && (k <= iter)
    x(k) = x(k-1) - (f(x(k-1))/df(x(k-1)));
    error(k) = abs(x(k)-x(k-1));
    k = k+1;
end
x = x(k-1);
end