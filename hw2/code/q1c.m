function [En] = q1c(n)

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

h = figure;
plot(X,Y);
xlabel('Value of x');
ylabel('Function Value');
name = strcat(int2str(n), 'function.fig');
savefig(h, name);

value = linspace(-1,1,200);
len = length(value);
functionVal = zeros(len,1);
actualFunctionVal = zeros(len,1);
E = zeros(len,1);
for i = 1:len
    [~,functionVal(i)] = q1(X,Y,value(i));
    actualFunctionVal(i) = 6 / (1+25*((value(i))^2));
    E(i) = abs(functionVal(i) - actualFunctionVal(i));
end

h = figure;
plot(value,E);
xlabel('Linearly Spaced Values of x');
ylabel('Error Value');
name = strcat(int2str(n), 'error.fig');
savefig(h, name);

En = max(E);
end