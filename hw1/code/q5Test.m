function q5Test

% run 1
n = 100; % number of points
err = zeros(n,1);
for i = 1:n
    P = rand(n,3);
    Q = rand(n,3);
    [ret_R, ret_t] = q5(P, Q);
    err(i) = q5CalcError(ret_R, ret_t, P, Q, n);
end

x = 1:n;
plot(x, err);
saveas(gcf, 'random-error.png');

% run 2
n = 100; % number of points
err = zeros(n,1);
for i = 1:n
    R = orth(rand(3,3));
    t = rand(3,1); 
    P = randn(n,3);
    Q = R*P' + repmat(t, 1, n);
    Q = Q';
    [ret_R, ret_t] = q5(P, Q);
    err(i) = q5CalcError(ret_R, ret_t, P, Q, n);
end

x = 1:n;
plot(x, err);
saveas(gcf, 'format-error.png');

end