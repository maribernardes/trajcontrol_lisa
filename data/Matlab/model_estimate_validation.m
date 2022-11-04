%With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');
clear; close all; clc;

%% Load Dataset
trial = 01;
folder = 'jacobian';
name = 'validate_J_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),'.mat'));

%% Configure simulation
alpha = 0.65; 
safe_limit = 6;
base_init = base(:,1);

%% Loop key
k_key = find(key); % Samples when key was pressed
ns = length(k_key);
step = 1:ns;
for i=1:ns
    k = k_key(i);   %sample
    Jc_step{i} = J{k};
    Z_step(:,i) = Z(:,k);
    X_step(:,i) = X(:,k);
end

%% Loop prediction steps
Z_hat(:,1) = Z_step(:,1);
err = zeros(5,ns);
err_3d = zeros(1,ns);
for i=2:ns    
    % Use Jacobian from experiment
    deltaZ_hat = Jc_step{i}*(X_step(:,i)-X_step(:,i-1)); % Predict estimate from Jacobian and inputs
    Z_hat(:,i) = deltaZ_hat + Z_step(:,i-1); 
    err(:,i) = Z_hat(:,i) - Z_step(:,i);
    err_3d(i) = sqrt(err(1,1)^2 + err(1,2)^2 + err(1,3)^2);
end

%% Calculate error

err_rms = rms(err,2);


fprintf('RMS error X [mm] = %0.4f\t Max err X [mm] = %0.4f\n', err_rms(1), max(err(1,:)));
fprintf('RMS error Y [mm] = %0.4f\t Max err Y [mm] = %0.4f\n', err_rms(2), max(err(2,:)));
fprintf('RMS error Z [mm] = %0.4f\t Max err Z [mm] = %0.4f\n', err_rms(3), max(err(3,:)));
fprintf('3D RMS error [mm] = %0.4f\n', rms(err_3d));


%% Plot insertion

figure  
plot(step, -X_step(1,:),'.-', step, X_step(3,:),'.-')
hold on
title('Needle guide position'),xlabel('step [k]'),ylabel('position [mm]'), legend('horizontal (X)','vertical (Z)')


figure  
plot(step, -Z_hat(1,:),'.-', step, -Z_step(1,:),'.-')
hold on
title('Tip position - Horizontal (X)'),xlabel('step [k]'),ylabel('X [mm]'), legend('estimated','measured')

figure  
plot(step, -Z_hat(2,:),'.-', step, -Z_step(2,:),'.-')
hold on
title('Tip position - Insertion (Y)'),xlabel('step [k]'),ylabel('Y [mm]'), legend('estimated','measured')


figure  
plot(step, Z_hat(3,:),'.-', step, Z_step(3,:),'.-')
hold on
title('Tip position - Vertical (Z)'),xlabel('step [k]'),ylabel('X [mm]'), legend('estimated','measured')

figure  
plot(step, err(1,:),'.-', step, err(2,:),'.-', step, err(3,:),'.-')
hold on
title('Estimation error'),xlabel('step [k]'),ylabel('Err [mm]'), legend('horizontal (X)', 'depth (Y)', 'vertical (Z)')



% 
% 
% plot3(Z_hat(1,:), Z_hat(2,:), Z_hat(3,:), '.-',  Z_step(1,:), Z_step(2,:), Z_step(3,:), '.-')
% hold on
% grid on
% title('Needle tip 3D position'),xlabel('X [mm]'),ylabel('Y [mm]'), zlabel('Z [mm]'), legend('estimated','measured')
% 
% % Plot results
% figure
% plot(Z_hat(1,:), Z_hat(3,:), '.-',  Z_step(1,:), Z_step(3,:), '.-')
% hold on
% grid on
% title('Needle tip 2D position'),xlabel('X [mm]'), ylabel('Z [mm]'), legend('estimated','measured')



% %disp("Error X [mm]");
% Xmean = mean(abs(Z_hat(1,1:N)-Z(1,1:N)));
% Xstd = std(abs(Z_hat(1,1:N)-Z(1,1:N)));
% Xmax = max(abs(Z_hat(1,1:N)-Z(1,1:N)));
% 
% %disp("Error Y [mm]");
% Ymean = mean(abs(Z_hat(2,1:N)-Z(2,1:N)));
% Ystd = std(abs(Z_hat(2,1:N)-Z(2,1:N)));
% Ymax = max(abs(Z_hat(2,1:N)-Z(2,1:N)));
% 
% % disp("Error Z [mm]");
% Zmean = mean(abs(Z_hat(3,1:N)-Z(3,1:N)));
% Zstd = std(abs(Z_hat(3,1:N)-Z(3,1:N)));
% Zmax = max(abs(Z_hat(3,1:N)-Z(3,1:N)));
% 
% % disp("Error Trajectory [mm]");
% Trajmean = mean(sqrt((Z_hat(1,1:N)-Z(1,1:N)).^2+(Z_hat(2,1:N)-Z(2,1:N)).^2+(Z_hat(3,1:N)-Z(3,1:N)).^2));
% Trajstd = std(sqrt((Z_hat(1,1:N)-Z(1,1:N)).^2+(Z_hat(2,1:N)-Z(2,1:N)).^2+(Z_hat(3,1:N)-Z(3,1:N)).^2));
% Trajmax = max(sqrt((Z_hat(1,1:N)-Z(1,1:N)).^2+(Z_hat(2,1:N)-Z(2,1:N)).^2+(Z_hat(3,1:N)-Z(3,1:N)).^2));
% 
%     
% fprintf('Error X [mm] = %0.4f +- %0.4f / max = %0.4f\n', Xmean, Xstd, Xmax);
% fprintf('Error Y [mm] = %0.4f +- %0.4f / max = %0.4f\n', Ymean, Ystd, Ymax);
% fprintf('Error Z [mm] %0.4f +- %0.4f / max = %0.4f\n', Zmean, Zstd, Zmax);
% fprintf('Error 3D[mm] %0.4f +- %0.4f / / max = %0.4f\n', Trajmean, Trajstd, Trajmax);
% 
% % 
% function plot_key(line)
%     global T
%     for i=1:length(T)
%         xline(T(i), line);
%     end
% end
