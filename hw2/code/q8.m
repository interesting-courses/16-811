function [final_alpha, interpolated_points, slopes, intercepts] = q8(p0)

fileId = fopen('../paths.txt', 'r');
formatSpec = '%f';
sizeA = [50 Inf];
A = fscanf(fileId, formatSpec, sizeA);
A = A.';

alpha = [-1 -1 -1]';
p0 = [p0 1]';

if p0(1) > p0(2)
    condn = 1;
else
    condn = 2;
end

x_coords = A(1:2:end, :);
y_coords = A(2:2:end, :);

i = 0;
paths = zeros(3,3);
while (alpha(1) < 0 || alpha(2) < 0 || alpha(3) < 0 || det(paths) == 0)
    [m,~] = size(x_coords);
    k = randperm(m);
    paths_x = x_coords(k(1:3),:);
    paths_y = y_coords(k(1:3),:);
    
    if(condn == 1)
        if (paths_x(1,1) == paths_y(1,1) || paths_x(2,1) == paths_y(2,1) || paths_x(3,1) == paths_y(3,1))
            if (paths_x(1,2) > paths_y(1,2) || paths_x(2,2) > paths_y(2,2) || paths_x(3,2) > paths_y(3,2))
                continue
            end
        end
        if(paths_x(1,1) < paths_y(1,1) || paths_x(2,1) < paths_y(2,1) || paths_x(3,1) < paths_y(3,1))
            continue
        end
    elseif (condn == 2)
        if (paths_x(1,1) == paths_y(1,1) || paths_x(2,1) == paths_y(2,1) || paths_x(3,1) == paths_y(3,1))
            if (paths_x(1,2) < paths_y(1,2) || paths_x(2,2) < paths_y(2,2) || paths_x(3,2) < paths_y(3,2))
                continue
            end
        end
        if(paths_x(1,1) > paths_y(1,1) || paths_x(2,1) > paths_y(2,1) || paths_x(3,1) > paths_y(3,1))
            continue
        end     
    end
    
    paths = [paths_x(1,1), paths_x(2,1), paths_x(3,1); paths_y(1,1), paths_y(2,1), paths_y(3,1); 1,1,1];
    alpha = paths\p0;
end

final_path_x = paths_x;
final_path_y = paths_y;
final_alpha = alpha;

interpolated_points = zeros(3,50);
[~,n] = size(final_path_x);

for i = 1:n
    A = [final_path_x(1,i), final_path_x(2,i), final_path_x(3,i); final_path_y(1,i), final_path_y(2,i), final_path_y(3,i); 1 1 1];
    interpolated_points(:,i) = A*alpha;
end

interpolated_points = interpolated_points(1:2,:);

slopes = zeros(n-1);
intercepts = zeros(n-1);
for i = 1:n-1
    points_1 = interpolated_points(:,i);
    points_2 = interpolated_points(:,i+1);
    slopes(i) = (points_2(2) - points_1(2))/(points_2(1) - points_1(1));
    intercepts(i) = (points_2(1)*points_1(2) - points_1(1)*points_2(2))/(points_2(1) - points_1(1));
end

slopes = slopes(:,1);
intercepts = intercepts(:,1);

x = interpolated_points(1,:);
y = interpolated_points(2,:);
plot(x,y,'r');
hold on

x = final_path_x(1,:);
y = final_path_y(1,:);
plot(x,y,'g');

x = final_path_x(2,:);
y = final_path_y(2,:);
plot(x,y,'g');

x = final_path_x(3,:);
y = final_path_y(3,:);
plot(x,y,'g');

ezplot('x^2+y^2-10*x-10*y+47.75',[0,12,0,12]);
title('Red Line is interpolated path');
hold off

end