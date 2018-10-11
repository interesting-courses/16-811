function value = q1b(n)

X = zeros(n+1,1);
for i = 0:n
    X(i+1) = i*(2/n) - 1;
end
X = X';

Y = zeros(n+1,1);
for i = 0:n
    Y(i+1) = 6 / (1+25*X(i+1)^2);
end
Y = Y';

[~,value] = q1(X,Y,0.05);

end