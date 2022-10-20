%With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');
clear; close all; clc;

%% Load Dataset
trial = 03;
folder = '2022-10-17';
name = 'exp_e2_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),'.mat'));

%% Configure simulation
alpha = 0.65; 
N = size(X,2);  % data size

% Smooth data even more
Xf = smoothdata(X,2);
Zf = smoothdata(Z,2);

%% Estimate Jacobian
% Initialize estimated needle tip
Z_hat = zeros(5,N);
Z_hat(:,1) = Z(:,1);
Z_hat(:,2) = Z(:,2);

% Estimate Jacobian recursively
num_magic = 0.0001;
Zant = Z(:,1);
Xant = X(:,1);
Tant = t(1);

% Select initial Jacobian
% Jsim = (Z(:,init+1)-Z(:,init))*pinv(X(:,init+1)-X(:,init));
Jsim = J{1};


%% Loop key
k_key = find(key); % Samples when key was pressed
Nk = length(k_key);
global T 
T = zeros(1,Nk);
for i=1:Nk
    k = k_key(i);   %sample
    T(:,i) = t(k);
end

%% Loop all measurements
for i=2:N
    if key(i) == 1 % key hit: Update Jacobian and calculate new Z_hat
        
        % Use Jacobian from experiment
        deltaZ_hat = J{i+1}*(X(:,i)-Xant); % Predict estimate from Jacobian and inputs

        Zant = Z(:,i);
        Xant = X(:,i);
        Tant = t(i);
        
        Z_hat(:,i) = deltaZ_hat + Z_hat(:,i-1); 

    else
        Z_hat(:,i) = Z_hat(:,i-1);  
    end 
end


%% Plot insertion

% Plot results
figure
plot3(Z_hat(1,:), Z_hat(2,:), Z_hat(3,:), '.-',  Z(1,:), Z(2,:), Z(3,:), '.-', Zf(1,:), Zf(2,:), Zf(3,:), '.-')
hold on
grid on
title('Needle tip position'),xlabel('X [mm]'),ylabel('Y [mm]'), zlabel('Z [mm]'), legend('estimated','measured', 'smoothed')

figure  
plot(t, Z_hat(1,:),'.-', t, Z(1,:), '.-', t, Zf(1,:), '.-')
hold on
plot_key('--g');
title('Tip position - X direction'),xlabel('time [s]'),ylabel('X [mm]'), legend('estimated','measured','smoothed')

figure  
plot(t, Z_hat(2,:), '.-', t, Z(2,:), '.-', t, Zf(2,:), '.-')
hold on
plot_key('--g');
title('Tip position - Y direction'),xlabel('time [s]'),ylabel('Y [mm]'), legend('estimated','measured','smoothed')

figure  
plot(t, Z_hat(3,:), '.-', t, Z(3,:), '.-', t, Zf(3,:), '.-')
hold on
plot_key('--g');
title('Tip position - Z direction'),xlabel('time [s]'),ylabel('Z [mm]'), legend('estimated','measured', 'smoothed')

figure  
plot(abs(Z(1,:)-Z_hat(1,:)),'.-'), title('Estimation error in position')
hold on
plot(abs(Z(2,:)-Z_hat(2,:)),'.-')
plot(abs(Z(3,:)-Z_hat(3,:)), '.-')
legend('X','Y','Z')
xlabel('sample #'),ylabel('Error [mm]')



%disp("Error X [mm]");
Xmean = mean(abs(Z_hat(1,1:N)-Z(1,1:N)));
Xstd = std(abs(Z_hat(1,1:N)-Z(1,1:N)));
Xmax = max(abs(Z_hat(1,1:N)-Z(1,1:N)));

%disp("Error Y [mm]");
Ymean = mean(abs(Z_hat(2,1:N)-Z(2,1:N)));
Ystd = std(abs(Z_hat(2,1:N)-Z(2,1:N)));
Ymax = max(abs(Z_hat(2,1:N)-Z(2,1:N)));

% disp("Error Z [mm]");
Zmean = mean(abs(Z_hat(3,1:N)-Z(3,1:N)));
Zstd = std(abs(Z_hat(3,1:N)-Z(3,1:N)));
Zmax = max(abs(Z_hat(3,1:N)-Z(3,1:N)));

% disp("Error Trajectory [mm]");
Trajmean = mean(sqrt((Z_hat(1,1:N)-Z(1,1:N)).^2+(Z_hat(2,1:N)-Z(2,1:N)).^2+(Z_hat(3,1:N)-Z(3,1:N)).^2));
Trajstd = std(sqrt((Z_hat(1,1:N)-Z(1,1:N)).^2+(Z_hat(2,1:N)-Z(2,1:N)).^2+(Z_hat(3,1:N)-Z(3,1:N)).^2));
Trajmax = max(sqrt((Z_hat(1,1:N)-Z(1,1:N)).^2+(Z_hat(2,1:N)-Z(2,1:N)).^2+(Z_hat(3,1:N)-Z(3,1:N)).^2));

    
fprintf('Error X [mm] = %0.4f +- %0.4f / max = %0.4f\n', Xmean, Xstd, Xmax);
fprintf('Error Y [mm] = %0.4f +- %0.4f / max = %0.4f\n', Ymean, Ystd, Ymax);
fprintf('Error Z [mm] %0.4f +- %0.4f / max = %0.4f\n', Zmean, Zstd, Zmax);
fprintf('Error 3D[mm] %0.4f +- %0.4f / / max = %0.4f\n', Trajmean, Trajstd, Trajmax);


function plot_key(line)
    global T
    for i=1:length(T)
        xline(T(i), line);
    end
end
