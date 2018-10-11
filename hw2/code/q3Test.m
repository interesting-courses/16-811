function x = q3Test

value = linspace(8,14,50);
x = zeros(length(value), 1);
f = 'x - tan(x)';
df = '1 - (sec(x))^2';
for i = 1:length(value)
    x(i) = q3(f, df, value(i));
end
end