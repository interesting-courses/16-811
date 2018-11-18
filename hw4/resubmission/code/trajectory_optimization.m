function trajectory_optimization(val, num_iter)

waypoints=300;
N=101;
OBST = [20,30;60,40;70,85];
epsilon = [25; 20; 30];

obs_cost = double(zeros(N));
for i=1:size(OBST,1)
    t = zeros(N);
    t(OBST(i,1),OBST(i,2)) = 1;
    t_cost = double(bwdist(t));
    t_cost(t_cost>epsilon(i))=epsilon(i);
    t_cost = 1/(2*epsilon(i))*(t_cost-epsilon(i)).^2;
    obs_cost = obs_cost + t_cost(1:N, 1:N);
end

figure(1);
imagesc(obs_cost');
% obstacle cost gradient
gx = diff(double(obs_cost),1,1);
gy = diff(double(obs_cost),1,2);
hold on;

figure(1);
surface(1:N,1:N,double(obs_cost'));
xlabel('X');
hold on;

%world params
SX = 10; % START
SY = 10;
GX = 90; % GOAL
GY = 90;

traj = zeros(2,waypoints);
traj(1,1)=SX;
traj(2,1)=SY;
dist_x = GX-SX;
dist_y = GY-SY;
for i=2:waypoints
    traj(1,i)=traj(1,i-1)+dist_x/(waypoints-1);
    traj(2,i)=traj(2,i-1)+dist_y/(waypoints-1);
end

% for i=2:150
%     traj(1,i)=traj(1,i-1);
%     traj(2,i)=traj(2,i-1)+dist_y/(149);
% end
% 
% for i=151:waypoints
%     traj(1,i)=traj(1,i-1)+dist_x/150;
%     traj(2,i)=traj(2,i-1);
% end

path_init = traj';
tt=size(path_init,1);
path_init_values = zeros(size(path_init,1),1);
for i=1:2
    path_init_values(i)=obs_cost(floor(path_init(i,1)),floor(path_init(i,2)));
end

figure(1);
plot3(path_init(:,1),path_init(:,2),path_init_values,'.r','MarkerSize',20);
hold on;

path = path_init;

% code starts here
[m,~] = size(path);
tol = 1e-3;
curr_norm = norm(path);
old_norm = 0;

% part a
if val == 1
    for i = 1:num_iter
        old_norm = norm(path);
        for k = 2:m-1
            x = floor(path(k,1));
            y = floor(path(k,2));
            path(k,1) = path(k,1) - 0.1*gx(x, y);
            path(k,2) = path(k,2) - 0.1*gy(x, y);
        end
        curr_norm = norm(path);
        if abs(curr_norm - old_norm) < tol
            break;
        end
    end
end

% part b
if val == 2
    for i = 1:num_iter
        for k = 2:m-1
            x = floor(path(k,1));
            y = floor(path(k,2));
            smooth_x = path(k,1) - path(k-1,1);
            smooth_y = path(k,2) - path(k-1,2);
            grad_x = 0.8*gx(x,y) + 4*smooth_x;
            grad_y = 0.8*gy(x,y) + 4*smooth_y;
            path(k,1) = path(k,1) - 0.1*grad_x;
            path(k,2) = path(k,2) - 0.1*grad_y;
        end
    end
end

% part c
if val == 3
    for i = 1:num_iter
        for k = 2:m-1
            x = floor(path(k,1));
            y = floor(path(k,2));
            smooth_x = 2*path(k,1) - path(k-1,1) - path(k+1,1);
            smooth_y = 2*path(k,2) - path(k-1,2) - path(k+1,2);
            grad_x = 0.8*gx(x,y) + 4*smooth_x;
            grad_y = 0.8*gy(x,y) + 4*smooth_y;
            path(k,1) = path(k,1) - 0.1*grad_x;
            path(k,2) = path(k,2) - 0.1*grad_y;
        end
    end
end
% code ends here

path_values = zeros(tt,1);
for i=1:tt
    path_values(i)=obs_cost(floor(path(i,1)),floor(path(i,2)));
end
figure(1)
hold on;
plot3(path(:,1),path(:,2),path_values,'.g','MarkerSize',20);

hold off;

end