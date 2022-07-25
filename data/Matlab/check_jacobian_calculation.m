%With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');
clear; close all; clc;

%% Configure simulation
alpha = 0.65; 


%% Load Dataset
trial = 03;
folder = '2022-07-20';
name = 'trialb_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),'.mat'));

% Size
N = size(X,2);

% % Smooth data even more
Xf = smoothdata(X,2);
Zf = smoothdata(Z,2);

%% Estimate Jacobian
% Initialize estimated needle tip
Z_hat = zeros(5,N);
Z_hat(:,1) = Z(:,1);
Z_hat(:,2) = Z(:,2);

Z_hat_sim = zeros(5,N);
Z_hat_sim(:,1) = Z(:,1);
Z_hat_sim(:,2) = Z(:,2);

% Estimate Jacobian recursively
num_magic = 0.0001;
Zant = Z(:,1);
Xant = X(:,1);
Tant = t(1);

%% Estimate Jacobian
% Initialize estimated needle tip
Z_hat = zeros(5,N);
Z_hat(:,1) = Z(:,1);
Z_hat(:,2) = Z(:,2);

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
        deltaZ_hat = J{i}*(X(:,i)-Xant); % Predict estimate from Jacobian and inputs

        % Use Jacobian calculated in matlab simulation
        deltaZsensor = (Z(:,i)-Zant);
        deltaXsensor = (X(:,i)-Xant);
        Jsim = Jsim + alpha*((deltaZsensor-Jsim*deltaXsensor)/(deltaXsensor'*deltaXsensor+num_magic))*deltaXsensor';

        deltaZ_hat_sim = Jsim*(X(:,i)-Xant); % Predict estimate from Jacobian and inputs

        Zant = Z(:,i);
        Xant = X(:,i);
        Tant = t(i);
        
%         Z_hat(:,i) = Z(:,i);
%         Z_hat_sim(:,i) = Z(:,i);
        Z_hat(:,i) = deltaZ_hat + Z_hat(:,i-1); 
        Z_hat_sim(:,i) = deltaZ_hat_sim + Z_hat_sim(:,i-1); 

    else
        deltaZ_hat = J{i}*(X(:,i)-X(:,i-1)); % Predict estimate from Jacobian and inputs
        Z_hat(:,i) = deltaZ_hat + Z_hat(:,i-1);  

        deltaZ_hat_sim = Jsim*(X(:,i)-X(:,i-1)); % Predict estimate from Jacobian and inputs
        Z_hat_sim(:,i) = deltaZ_hat_sim + Z_hat_sim(:,i-1);       

    end 
end


%% Plot results
figure  
plot(abs(Z_hat(1,:)-Z_hat_sim(1,:)),'.-'), title('Difference between simulated and experiment')
hold on
plot(abs(Z_hat(2,:)-Z_hat_sim(2,:)),'.-')
plot(abs(Z_hat(3,:)-Z_hat_sim(3,:)), '.-')
plot_key('--g');
legend('X','Y','Z')
xlabel('sample #'),ylabel('Error [mm]')

figure  
plot(t, Z_hat(1,:), '.-', t, Z_hat_sim(1,:), '.-', t, Z(1,:), '.-')
hold on
plot_key('--g');
title('Tip position - X direction'),xlabel('time [s]'),ylabel('X [mm]'), legend('experiment', 'simulation', 'measured')

figure  
plot(t, Z_hat(2,:), '.-', t, Z_hat_sim(2,:), '.-', t, Z(2,:), '.-')
hold on
plot_key('--g');
title('Tip position - Y direction'),xlabel('time [s]'),ylabel('Y [mm]'), legend('experiment', 'simulation', 'measured')

figure  
plot(t, Z_hat(3,:), '.-', t, Z_hat_sim(3,:), '.-',t, Z(3,:), '.-')
hold on
plot_key('--g');
title('Tip position - Z direction'),xlabel('time [s]'),ylabel('Z [mm]'), legend('experiment', 'simulation','measured')


figure  
plot(abs(Z(1,:)-Z_hat(1,:)),'.-'), title('Experiment estimation error in position')
hold on
plot(abs(Z(2,:)-Z_hat(2,:)),'.-')
plot(abs(Z(3,:)-Z_hat(3,:)), '.-')
plot_key('--g');
legend('X','Y','Z')
xlabel('sample #'),ylabel('Error [mm]')

figure  
plot(abs(Z(1,:)-Z_hat_sim(1,:)),'.-'), title('Simulation estimation error in position')
hold on
plot(abs(Z(2,:)-Z_hat_sim(2,:)),'.-')
plot(abs(Z(3,:)-Z_hat_sim(3,:)), '.-')
plot_key('--g');
legend('X','Y','Z')
xlabel('sample #'),ylabel('Error [mm]')


function plot_key(line)
    global T
    for i=1:length(T)
        xline(T(i), line);
    end
end

% 
% %disp("Error X [mm]");
% Xmean = mean(abs(Z_hat(1,init:N)-Z(1,init:N)));
% Xmax = max(abs(Z_hat(1,init:N)-Z(1,init:N)));
% 
% %disp("Error Y [mm]");
% Ymean = mean(abs(Z_hat(2,init:N)-Z(2,init:N)));
% Ymax = max(abs(Z_hat(2,init:N)-Z(2,init:N)));
% 
% % disp("Error Z [mm]");
% Zmean = mean(abs(Z_hat(3,init:N)-Z(3,init:N)));
% Zmax = max(abs(Z_hat(3,init:N)-Z(3,init:N)));
% 
% % disp("Error Trajectory [mm]");
% Trajmean = mean(sqrt((Z_hat(1,init:N)-Z(1,init:N)).^2+(Z_hat(2,init:N)-Z(2,init:N)).^2+(Z_hat(3,init:N)-Z(3,init:N)).^2));
% Trajmax = max(sqrt((Z_hat(1,init:N)-Z(1,init:N)).^2+(Z_hat(2,init:N)-Z(2,init:N)).^2+(Z_hat(3,init:N)-Z(3,init:N)).^2));
% 
%     
% fprintf('Error X [mm]\n mean = %0.4f / max = %0.4f\n', Xmean, Xmax);
% fprintf('Error Y [mm]\n mean = %0.4f / max = %0.4f\n', Ymean, Ymax);
% fprintf('Error Z [mm]\n mean = %0.4f / max = %0.4f\n', Zmean, Zmax);
% fprintf('Error trajectory [mm]\n mean = %0.4f / max = %0.4f\n', Trajmean, Trajmax);