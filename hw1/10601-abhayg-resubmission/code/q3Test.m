function q3Test(A,b)

[xbar, result] = q3(A,b);
if (result == 1)
    fprintf('System of equations has %d solution \n', result);
else
    fprintf('System of equations has %d solutions \n', result);
end

fprintf('Solution is')
xbar