function [table_dd, functionVal] = q1(X,Y,value)
% X = input vector of different x values
% Y = input vector of different f(x) values
% sum = interpolated function at x = value

if nargin ~= 3
    error('Must have x, f(x) and interpolation value');
end

[n,m] = size(X);

if n~= 1 && m~=size(Y,2)
    error('X and Y must have the same points');
end
    
table_dd = zeros(m,m);
table_dd(:,1) = Y';
for j = 2:m
    for i = 1:(m-j+1)
        table_dd(i,j) = (table_dd(i + 1, j - 1) - table_dd(i, j - 1)) / (X(i + j - 1) - X(i));
    end
end

coefficients = table_dd(1,:);
sum = coefficients(1);
for i = 2:length(coefficients)
    new_product = 1;
    for j = 1:i-1
        new_product = new_product*(value-X(j));
    end
    sum = sum + coefficients(i)*new_product;
end
functionVal = sum;

stringsum = num2str(coefficients(1));
for i = 2:length(coefficients)
    stringnew_product = "";
    for j = 1:i-1
        stringtemp = strcat('(x-', num2str(X(j)), ')');
        stringnew_product = strcat(stringnew_product, stringtemp);
    end
    stringsum = strcat(stringsum, '+');
    stringsum = strcat(stringsum, num2str(coefficients(i)), '*', new_product);
end

% display(stringsum);
end