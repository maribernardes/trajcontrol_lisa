clear; close all; clc;
global target;
global safe_limit;
global base_init;

INSERTION_STEP = -5;

%% Load Dataset
trial = 01;
extra = '';
folder = '2022-08-31';
name = 'trialk_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),extra,'.mat'));

%% Configure simulationb
S = size(up,2);  % data size
safe_limit = 6;
base_init = base(:,1);

%% Loop key
k_key = find(key); % Samples when key was pressed
ns = length(k_key);
T = zeros(1,ns);
for i=1:ns
    k = k_key(i);   %sample
    Jc_step{i} = J{k}(1:3,:);
    Z_step(i,:) = Z(:,k);
    X_step(i,:) = X(:,k);
    cmd_step(i,:) = cmd(:, k);
    if size(yp{1},2) == 3
        err_step(i,:) = Z(1:3,k) - target;
    else
        err_step(i,:) = Z(1:5,k) - [target;0;0];
    end
end

fprintf('Final error X [mm] = %0.4f\n', abs(err_step(end,1)));
fprintf('Final error Y [mm] = %0.4f\n', abs(err_step(end,2)));
fprintf('Final error Z [mm] = %0.4f\n', abs(err_step(end,3)));

if size(err_step,2) == 5
    fprintf('Final error angle_v [rad] = %0.4f\n', abs(err_step(end,4)));
    fprintf('Final error angle_h [rad] = %0.4f\n', abs(err_step(end,5)));
end
fprintf('Final error 3D[mm] = %0.4f\n', sqrt(err_step(end,1)^2+err_step(end,2)^2+err_step(end,3)^2));
fprintf('Final error 2D[mm] = %0.4f\n', sqrt(err_step(end,1)^2+err_step(end,3)^2));

depth = [0:-5:-100];

% First insertion step
subplot(4,1,1);
plot(depth(1:2), Z_step(1:2,1), '.-m');
plot_target_X('--k');
title('X - Horizontal');
ylabel('Tip X [mm]'); legend('executed', 'target');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,2);
plot(depth(1:2), X_step(1:2,1), '.-m');
plot_safe_limit_X('k');
ylabel('Base X [mm]'); legend('executed', 'safe limit');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,3);
plot(depth(1:2), Z_step(1:2,3), '.-m');
plot_target_Z('--k');
title('Z - Vertical');
ylabel('Tip Z [mm]'); legend('executed', 'target');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,4);
plot(depth(1:2), X_step(1:2,3), '.-m');
plot_safe_limit_Z('k');
xlabel('Depth [mm]'),ylabel('Base Z [mm]'); legend('executed', 'safe limit');
set(gca,'Xdir','reverse');
xlim([-100 0]);

pause;

past_u = [X_step(1:2,1) X_step(1:2,3)];
past_y = Z_step(1:2,1:3);

future_u = [past_u(2,:); up{1}];
future_y = [past_y(2,:); yp{1}(:,1:3)];

%% Loop prediction steps
for i=2:S    
    H = min(S-i+1,size(up{i},1));
    subplot(4,1,1);
    plot(depth(1:i), past_y(:,1), '.-m', depth(i:H+i), future_y(:,1), '.-b');
    plot_target_X('--k');
    title('X - Horizontal');
    ylabel('Tip X [mm]'); legend('executed','prediction',  'target');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);
    
    subplot(4,1,2);
    plot(depth(1:i), past_u(:,1), '.-m', depth(i:H+i), future_u(:,1), '.-b');
    plot_safe_limit_X('k');
    ylabel('Base X [mm]'); legend('executed','prediction',  'safe limit');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);

    
    subplot(4,1,3);
    plot(depth(1:i), past_y(:,3), '.-m', depth(i:H+i), future_y(:,3), '.-b');
    plot_target_Z('--k');
    title('Z - Vertical');
    ylabel('Tip Z [mm]'); legend('executed','prediction',  'target');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);
    
    subplot(4,1,4);
    plot(depth(1:i), past_u(:,2), '.-m', depth(i:H+i), future_u(:,2), '.-b');
    plot_safe_limit_Z('k');
    xlabel('Depth [mm]'),ylabel('Base Z [mm]'); legend('executed', 'prediction', 'safe limit');
    set(gca,'Xdir','reverse');
    xlim([-100 0]);

    if (i~=S)
        futH = min(S-i ,size(up{i},1));
        future_u = [X_step(i+1,1), X_step(i+1,3); up{i}(1:futH,:)];
        future_y = [Z_step(i+1,1:3); yp{i}(1:futH,1:3)];
    end
    past_u = [past_u; [X_step(i+1,1) X_step(i+1,3)]];
    past_y = [past_y; Z_step(i+1,1:3)];

    pause;
    
end

subplot(4,1,1);
plot(depth, Z_step(:,1), '.-m');
plot_target_X('--k');
title('X - Horizontal');
ylabel('Tip X [mm]'); legend('executed', 'target');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,2);
plot(depth, X_step(:,1), '.-m');
plot_safe_limit_X('k');
ylabel('Base X [mm]'); legend('executed', 'safe limit');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,3);
plot(depth, Z_step(:,3), '.-m');
plot_target_Z('--k');
title('Z - Vertical');
ylabel('Tip Z [mm]'); legend('executed', 'target');
set(gca,'Xdir','reverse');
xlim([-100 0]);

subplot(4,1,4);
plot(depth, X_step(:,3), '.-m');
plot_safe_limit_Z('k');
xlabel('Depth [mm]'),ylabel('Base Z [mm]'); legend('executed', 'safe limit');
set(gca,'Xdir','reverse');
xlim([-100 0]);

function plot_target(line)
    global target;
    plot_target_X(line);
    plot_target_Z(line);
end

function plot_target_X(line)
    global target;
    yline(target(1), line);
    ylim([target(1)-15, target(1)+15])
end

function plot_target_Z(line)
    global target;
    yline(target(3), line);
    ylim([target(3)-15, target(3)+15])
end

function plot_safe_limit(line)
    global base_init;
    global safe_limit;
    plot_safe_limit_X(line);
    plot_safe_limit_Z(line);
end

function plot_safe_limit_X(line)
    global base_init;
    global safe_limit;
    yline(base_init(1)+safe_limit,line);
    yline(base_init(1)-safe_limit,line);
    ylim([base_init(1)-safe_limit-3, base_init(1)+safe_limit+3])

end

function plot_safe_limit_Z(line)
    global base_init;
    global safe_limit;
    yline(base_init(3)+safe_limit,line);
    yline(base_init(3)-safe_limit,line);
    ylim([base_init(3)-safe_limit-3, base_init(3)+safe_limit+3])
end