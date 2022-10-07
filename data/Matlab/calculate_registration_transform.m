% Registers two sets of 3DoF data
% Assumes A and B are d,n sets of data
% where d is the dimension of the system 
% typically d = 2,3
% and n is the number of points
% typically n>3
%
% Mili Shah
% July 201402
clear all
close all 
clc

%% Get .csv file into table T
% Filename
folder = 'New registration';
name = 'registration_points_all';

% Read data from table
T = readtable(strcat('../../files/', folder,'/', name,'.csv'), 'HeaderLines',0);

%% Extract values from table
n = width(T);                   
A_orig = T{1:3,:}; %Aurora
B_orig = T{4:6,:}; %Robot

%% Calculate registration transform
%Mean Center Data
Ac = mean(A_orig,2);
Bc = mean(B_orig,2);
A = A_orig-repmat(Ac,1,n);
B = B_orig-repmat(Bc,1,n);

%Calculate Optimal Rotation
M = A*B';
N = [M(1,1)+M(2,2)+M(3,3)  M(2,3)-M(3,2)          M(3,1)-M(1,3)          M(1,2)-M(2,1);...
     M(2,3)-M(3,2)         M(1,1)-M(2,2)-M(3,3)   M(1,2)+M(2,1)          M(3,1)+M(1,3);...
     M(3,1)-M(1,3)         M(1,2)+M(2,1)         -M(1,1)+M(2,2)-M(3,3)   M(2,3)+M(3,2);...
     M(1,2)-M(2,1)         M(3,1)+M(1,3)          M(2,3)+M(3,2)         -M(1,1)-M(2,2)+M(3,3)];
[u,v] = eig(N);
[m,ind] = max(diag(v));
q = u(:,ind);
R = (q(1)^2-q(2:4)'*q(2:4))*eye(3) + 2*q(2:4)*q(2:4)'+2*q(1)*[0 -q(4) q(3); q(3) 0 -q(2); -q(3) q(2) 0];
%Calculate Optimal Translation
t = Bc - R*Ac;

% % Pedro - A (similarity)
% q3dslicer = [0.7241; 0.6891; -0.0070; -0.0279];
% t3dslicer = [-114.67; -261.34; 10.69];

% % Pedro - A (rigid)
% q3dslicer = [0.7242; 0.6889; -0.0070; -0.0282];
% t3dslicer = [-113.45; -257.45; 11.02];

% % Mariana - B and C (similarity)
% rotm = [1.02482 0.0146421 -0.0163364; -0.0168445 0.0361643 -1.02428; -0.0140546 1.02431 0.0363966];
% q3dslicer = [0.7195; 0.6944; -0.0008; -0.0107];
% t3dslicer = [-110.998; -265.685; 5.59876];

% % Mariana - B and C (rigid)
% rotm = [0.999771 0.0142842 -0.0159371; -0.0164328 0.0352804 -0.999242; -0.0137111 0.999275 0.035507];
% q3dslicer =  [0.7195; 0.6944; -0.0008; -0.0107];
% t3dslicer = [-108.896; -259.191; 6.31788];

% % Mariana - B,C,D,E (similarity)
% rotm = [1.02444 0.0149652 -0.0161876; -0.0166984 0.0355228 -1.02393; -0.014393 1.02396 0.0357585];
% q3dslicer =  [0.7193; 0.6947; -0.0006; -0.0107];
% t3dslicer = [-110.912; -265.55; 5.53846];

% Mariana - B,C,D,E (rigid)
rotm = [0.999769 0.0146047 -0.0157977; -0.0162962 0.0346672 -0.999266; -0.0140463 0.999292 0.0348972];
q3dslicer =  [0.7193; 0.6947; -0.0006; -0.0107];
t3dslicer = [-108.842; -259.158; 6.2481];

t = t3dslicer;
q = q3dslicer;

new_regist_tf = [t;q];
csvwrite(strcat('../../files/', folder,'/registration_matlab.csv'),new_regist_tf)


qr = quaternion(q(1), q(2), q(3), q(4));
qt = quaternion(0, t(1), t(2), t(3));
p_A_orig = quaternion(zeros(1,n), A_orig(1,:), A_orig(2,:), A_orig(3,:));
p_B_orig = quaternion(zeros(1,n), B_orig(1,:), B_orig(2,:), B_orig(3,:));

% Register points
for i=1:n
    p_B(i) = qr*p_A_orig(i)*conj(qr) + qt;
    p_A(i) = conj(qr)*(p_B_orig(i) - qt)*qr;
end

% Aurora registered to robot
B_reg = compact(p_B)';
B_reg = B_reg(2:4,:);

% Robot registered to aurora
A_reg = compact(p_A)';
A_reg = A_reg(2:4,:);

figure(1);
plot3(A_orig(1,:), A_orig(2,:), A_orig(3,:), 'r');
hold on;
xlabel('X'); 
ylabel('Y');
zlabel('Z');
% plot3(B_orig(1,:), B_orig(2,:), B_orig(3,:), 'b');
% legend({'robot','aurora'})
axis equal

figure (2);
plot3(B_orig(1,:), B_orig(2,:), B_orig(3,:), 'o-b');
hold on;
xlabel('X'); 
ylabel('Y');
zlabel('Z');
plot3(B_reg(1,:), B_reg(2,:), B_reg(3,:), 'o-r');
legend({'robot','aurora to robot'})
axis equal

figure(3);
plot3(A_orig(1,:), A_orig(2,:), A_orig(3,:), 'o-r');
hold on;
xlabel('X'); 
ylabel('Y');
zlabel('Z');
plot3(A_reg(1,:), A_reg(2,:), A_reg(3,:), 'o-b');
legend({'aurora','robot to aurora'})
axis equal

figure(4);
plot(B_orig(1,:), B_orig(3,:), 'o-b');
hold on;
xlabel('X'); 
ylabel('Z');
plot(B_reg(1,:), B_reg(3,:), 'o-r');
legend({'robot','aurora to robot'})
axis equal
