function En = q1cTest

% n_val = [2,4,6,8,10,12,14,16,18,20,40];
n_val = [40];
En = zeros(length(n_val),1);
for i = 1:length(n_val)
    En(i) = q1c(n_val(i));
end

plot(n_val, En);
xlabel('Value of n');
ylabel('Error Estimate');
end