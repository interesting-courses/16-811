function px = q1bTest

n_val = [2,4,40];
px = zeros(length(n_val),1);
for i = 1:length(n_val)
  px(i) = q1b(n_val(i));
end
end