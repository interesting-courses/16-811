function [alpha, point_t] = q8Test(p0, t)

% p0 = [0.8 1.8];
[final_alpha, interpolated_points, slopes, intercepts] = q8(p0);
alpha = final_alpha;
floor_t = floor(t);
ceil_t = ceil(t);
ratio_t = t-floor_t;

x_t = interpolated_points(1,ceil_t) + ratio_t*(interpolated_points(1,ceil_t+1) - interpolated_points(1,ceil_t));
y_t = slopes(ceil_t)*x_t + intercepts(ceil_t);

point_t = [x_t, y_t];

end