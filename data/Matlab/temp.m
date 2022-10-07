clear all;
close all;
clc;
% %% Load Dataset
% trial = 0;
% extra = 'g';
% folder = '2022-10-04';
% name = 'trialr_';
% load(strcat(folder,'/',name,num2str(trial,'%2.2d'),extra,'.mat'));
% 
% target

pos_z = [23.9700000000000;30.3474000000000;35.1923000000000;18.7643000000000;37.7225000000000;33.1670000000000;29.0431000000000;27.8808000000000;28.0568000000000;27.7555000000000;22.7002000000000;27.1150000000000;18.8482000000000;12.9038000000000;9.31080000000000;16.7289000000000;9.94480000000000;16.3738000000000;24.1666000000000;31.5673000000000];
err_2d = [0.575600000000000;0.406700000000000;0.327700000000000;0.716900000000000;0.731900000000000;1.14080000000000;1.50580000000000;0.657400000000000;1.24740000000000;0.0517000000000000;0.976700000000000;0.532800000000000;0.377400000000000;0.506800000000000;0.504700000000000;0.594700000000000;0.317400000000000;0.473000000000000;0.269100000000000;0.356000000000000];
N = length(err_2d);

figure(1);
text(pos_z+0.5*ones(N,1), err_2d, string(1:N));
hold on;
plot(pos_z, err_2d, '.b');
title('2D Error with X Position = -25 mm')
box on
xlabel('Z Position [mm]'); 
ylabel('Err 2D [mm]'); 
