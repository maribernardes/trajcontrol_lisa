%% Build Jacobian control matrices (for each step)
clear; close all; clc;

%% Load Dataset
trial = 01;
folder = '2022-08-17';
name = 'trialh_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),'.mat'));


%% Loop key
k_key = find(key); % Samples when key was pressed
ns = length(k_key);
T = zeros(1,ns);
for i=1:ns
    k = k_key(i);   %sample
    Jc_exp{i} = J{k}(1:3,:);
    Z_exp(i,:) = Z(:,k);
    X_exp(i,:) = X(:,k);
    cmd_exp(i,:) = cmd(:, k+1);
end
