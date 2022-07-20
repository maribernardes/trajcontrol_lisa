clear; close all; clc;

%% Configure simulation
trial = 0;
alpha = 0.65; 
sigma = 0*[0.05; 0.05; 0.05; 0.001; 0.001];  % measurement noise standard deviation
updateRate = 50; %How many estimated before updating with sensor readings

% Plots
plotflag = 1;   %Plot graphs? 1=YES / 0=NO
Z_hatflag = 1;  %Update tip estimate? 1=YES / 0=NO

fprintf('**********************************************\n');
fprintf('With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');
fprintf('Trial #%i\n', trial);
fprintf('Alpha = %0.4f\n', alpha);
fprintf('Update Rate = %i\n\n', updateRate);

%% Load Dataset
load(strcat('trial_',num2str(trial,'%2.2d'),'.mat'));

% Size
N = size(X,2);

% % Smooth data even more
X = smoothdata(X,2);
Z = smoothdata(Z,2);
T = t;

init = 10; %starting sample

%% Estimate Jacobian
% Initialize estimated needle tip
Z_hat = zeros(5,N);
Z_hat(:,init) = Z(:,init);
Z_hat(:,init+1) = Z(:,init+1);

% Estimate Jacobian recursively
num_magic = 0.0001;
Zant = Z(:,init);
Xant = X(:,init);
Tant = T(init);

% Select initial Jacobian
Jsim = (Z(:,init+1)-Z(:,init))*pinv(X(:,init+1)-X(:,init));


for i=(init+2):N
    if mod(i,updateRate)==0 % Correct Jacobian estimate
        
        Zsensor = Z(:,i) + diag(sigma)*randn(5,1);
        
        deltaT = T(i)-Tant;
        deltaZsensor = (Zsensor-Zant)/deltaT;
        deltaXsensor = (X(:,i)-Xant)/deltaT;
                
        Jsim = Jsim + alpha*((deltaZsensor-Jsim*deltaXsensor)/(deltaXsensor'*deltaXsensor+num_magic))*deltaXsensor';        
        deltaZ_hat = Jsim*(X(:,i)-Xant); % Predict estimate from Jacobian and inputs


        Zant = Zsensor;
        Xant = X(:,i);
        Tant = T(i);
        
        if (Z_hatflag == 1)
            Z_hat(:,i) = Zsensor;
        else
            Z_hat(:,i) = deltaZ_hat + Z_hat(:,i-1); 
        end

    else
        deltaZ_hat = Jsim*(X(:,i)-X(:,i-1)); % Predict estimate from Jacobian and inputs
        Z_hat(:,i) = deltaZ_hat + Z_hat(:,i-1);  
    end
      
end


%% Plot insertion

% Plot results
if plotflag 
    figure
    plot3(Z_hat(1,init:N), Z_hat(2,init:N), Z_hat(3,init:N), '.-', Z(1,init:N), Z(2,init:N), Z(3,init:N), '.-')
    hold on
    grid on
    title('Needle tip position'),xlabel('X [mm]'),ylabel('Y [mm]'), legend('estimated','measured')

    figure  
    plot(T(init:N), Z_hat(1,init:N),'.-', T(init:N), Z(1,init:N), '.-')
    title('Tip position - X direction'),xlabel('time [s]'),ylabel('X [mm]'), legend('estimated','measured')

    figure  
    plot(T(init:N), Z_hat(2,init:N), '.-', T(init:N), Z(2,init:N), '.-')
    title('Tip position - Y direction'),xlabel('time [s]'),ylabel('Y [mm]'), legend('estimated','measured')

    figure  
    plot(T(init:N), Z_hat(3,init:N), '.-', T(init:N), Z(3,init:N), '.-')
    title('Tip position - Z direction'),xlabel('time [s]'),ylabel('Z [mm]'), legend('estimated','measured')

    figure  
    plot(abs(Z(1,init:N)-Z_hat(1,init:N)),'.-'), title('Estimation error in position')
    hold on
    plot(abs(Z(2,init:N)-Z_hat(2,init:N)),'.-')
    plot(abs(Z(3,init:N)-Z_hat(3,init:N)), '.-')
    legend('X','Y','Z')
    xlabel('sample #'),ylabel('Error [mm]')

end

%disp("Error X [mm]");
Xmean = mean(abs(Z_hat(1,init:N)-Z(1,init:N)));
Xmax = max(abs(Z_hat(1,init:N)-Z(1,init:N)));

%disp("Error Y [mm]");
Ymean = mean(abs(Z_hat(2,init:N)-Z(2,init:N)));
Ymax = max(abs(Z_hat(2,init:N)-Z(2,init:N)));

% disp("Error Z [mm]");
Zmean = mean(abs(Z_hat(3,init:N)-Z(3,init:N)));
Zmax = max(abs(Z_hat(3,init:N)-Z(3,init:N)));

% disp("Error Trajectory [mm]");
Trajmean = mean(sqrt((Z_hat(1,init:N)-Z(1,init:N)).^2+(Z_hat(2,init:N)-Z(2,init:N)).^2+(Z_hat(3,init:N)-Z(3,init:N)).^2));
Trajmax = max(sqrt((Z_hat(1,init:N)-Z(1,init:N)).^2+(Z_hat(2,init:N)-Z(2,init:N)).^2+(Z_hat(3,init:N)-Z(3,init:N)).^2));

    
fprintf('Error X [mm]\n mean = %0.4f / max = %0.4f\n', Xmean, Xmax);
fprintf('Error Y [mm]\n mean = %0.4f / max = %0.4f\n', Ymean, Ymax);
fprintf('Error Z [mm]\n mean = %0.4f / max = %0.4f\n', Zmean, Zmax);
fprintf('Error trajectory [mm]\n mean = %0.4f / max = %0.4f\n', Trajmean, Trajmax);

function x_new = pose_transform(x_orig, x_tf)

    %Define frame transformation
    p_tf = quaternion(0, x_tf(1), x_tf(2), x_tf(3));
    q_tf= quaternion(x_tf(4), x_tf(5), x_tf(6), x_tf(7));

    %Define original position and orientation
    p_orig = quaternion(0, x_orig(1), x_orig(2), x_orig(3));
    q_orig = quaternion(x_orig(4), x_orig(5), x_orig(6), x_orig(7));

    %Transform to new frame
    q_new = q_tf*q_orig;
    p_new = q_tf*p_orig*conj(q_tf) + p_tf;

    vec_new = compact(p_new);
    x_new = [vec_new(2:4) compact(q_new)]';
end