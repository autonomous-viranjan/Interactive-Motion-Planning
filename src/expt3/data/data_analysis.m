% Viranjan Bhattacharyya
% EMC2 Lab Clemson University

clear, clc, close all
%%
subject = 'jh1';
key = append(subject, '.txt');
log_data = load(key);
%%
start = 12;
fin = length(log_data);
X = log_data(start:fin, 1:5);
X_ref = log_data(start:fin, 7:11);
Ua = log_data(start:fin, 12);
Ul = log_data(start:fin, 13);
X_nv = log_data(start:fin, 15:18);
X_obs = log_data(start:fin, 20);
alpha_v = log_data(start:fin, 22);
alpha_a = log_data(start:fin, 23);
X_nv_pred = log_data(start:fin, 25:27);
roadlength = 140;
%%
figure(1)
set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5],'Ylim',[0 roadlength])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
plot(X_ref(:,4),X_ref(:,1),'-.k')
plot(X(:,4),X(:,1),'b')
plot(X_nv(:,4),X_nv(:,1),'r')
rectangle('Position',[0.5 (X_obs(1)-5) 0.5 5],'FaceColor',[1 0.8 0.8])
xlabel('Lane number')
ylabel('Road length [m]')
legend('','Reference','Tracked','NV')
%%
figure(2)
set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5],'Ylim',[0 roadlength])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
l = X(:,4)';
s = X(:,1)';
z = zeros(size(s));
col = 1:(fin - start + 1);
surface([l;l],[s;s],[z;z],[col;col],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',4);
l_nv = X_nv(:,4)';
s_nv = X_nv(:,1)';
surface([l_nv;l_nv],[s_nv;s_nv],[z;z],[col;col],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
rectangle('Position',[0.5 (X_obs(1)-5) 0.5 5],'FaceColor',[1 0.9 0.9])
xlabel('Lane number')
ylabel('Road length [m]')
legend('', 'Ego', 'NV')
title('Trajectory')
%%
RMSE = rmse(X_ref, X);
%%
figure(3)
subplot(211)
plot(Ua,'g')
hold on
plot(X(:,3));
ylabel('m/s^2')
legend('Command acceleration','Ego acceleration')
subplot(212)
plot(Ul,'g')
hold on
plot(X(:,4))
ylabel('Lane')
legend('Command lane', 'Ego')
xlabel('Time step')
%%
figure(4)
plot(Ua, 'g')
hold on
plot(X(:,3))
plot(X_ref(:,3))
ylim([-2 2])
ylabel('m/s^2')
legend('Commanded acceleration','Ego acceleration','Reference acceleration')
xlabel('Time step')
%%
figure(5)
subplot(211)
plot(X(:,2))
hold on
plot(X_nv(:,2))
title('Velocity')
ylabel('[m/s]')
legend('Ego','NV')
subplot(212)
plot(alpha_v,'r')
hold on
plot(alpha_a,'g')
legend('$\alpha_v$','$\alpha_a$','Interpreter','Latex')
title('Imputation')
xlabel('Time step')
%%
figure(6)
plot(X(:,1))
hold on
plot(X_nv(:,1))
legend('Ego','NV')
ylabel('Longitudinal Position [m]')
xlabel('Time step')
%%
figure(7)
plot(X(:,4))
hold on
plot(X_nv(:,4))
legend('Ego','NV')
ylabel('Lateral Position [m]')
xlabel('Time step')
%%
figure(8)
set (gca,'DataAspectRatio',[1 60 1],'Xdir','reverse','Xlim',[1.5 2.5])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
plot(X_nv(:,4),X_nv(:,1),'Color',[0.8 0.1 0.5],'LineWidth',2)
plot(2*ones(length(X_nv_pred)), X_nv_pred(:,1),'.r')
xlabel('Lane number')
ylabel('Road length [m]')
legend('','Actual','Predicted')
title('NV actual and predicted trajectory')
%%
figure(9)
plot(X_nv(:,2))
hold on
plot(X_nv_pred(:,2))
title('Velocity')
ylabel('[m/s]')
legend('NV actual','NV predicted')
%% Horizon data
N = 20;
dt = 0.05;
key = append('horizon', subject, '.txt');
horizon_data = load(key);
horizon_data = horizon_data((start-1)*N+1:end, :);
if fin == length(horizon_data)/N
    Tsim = length(start:fin);
else
    Tsim = length(horizon_data)/N;
end

s = zeros(Tsim, N);
v = zeros(Tsim, N);
a = zeros(Tsim, N);
l = zeros(Tsim, N);
rl = zeros(Tsim, N);

s_nv = zeros(Tsim, N);
v_nv = zeros(Tsim, N);
a_nv = zeros(Tsim, N);

time_step = 1:Tsim;
step_matrix = zeros(length(time_step), N);
for i=1:length(time_step)
    for j=1:N
        step_matrix(i, j) = time_step(i) + (j-1);
    end
end
%%
for t = 1:Tsim
    s(t, :) = horizon_data((t-1)*N+1:N*t, 1);
    v(t, :) = horizon_data((t-1)*N+1:N*t, 2);
    a(t, :) = horizon_data((t-1)*N+1:N*t, 3);
    l(t, :) = horizon_data((t-1)*N+1:N*t, 4);
    rl(t, :) = horizon_data((t-1)*N+1:N*t, 5);

    s_nv(t, :) = horizon_data((t-1)*N+1:N*t, 6);
    v_nv(t, :) = horizon_data((t-1)*N+1:N*t, 7);
    a_nv(t, :) = horizon_data((t-1)*N+1:N*t, 8);
end

% NV position pred plot
figure(100)
hold on
for i = 1:Tsim
    plot(step_matrix(i, :), s_nv(i, :))
end
% actual position line
plot(X_nv(:, 1), 'b', 'LineWidth',2)
xlabel('Time step')
ylabel('NV Position [m]')
title('NV Position Prediction')

% NV velocity pred plot
figure(101)
hold on
for i = 1:Tsim
    plot(step_matrix(i, :), v_nv(i, :))
end
% actual position line
plot(X_nv(:, 2), 'b', 'LineWidth',2)
xlabel('Time step')
ylabel('NV Velocity [m/s]')
title('NV Velocity Prediction')

% NV acceleration pred plot
figure(102)
hold on
for i = 1:Tsim
    plot(step_matrix(i, :), a_nv(i, :))
end
% actual position line
plot(X_nv(:, 3), 'b', 'LineWidth',2)
xlabel('Time step')
ylabel('NV Acceleration [m/s^2]')
title('NV Acceleration Prediction')

%% Lower density plots
step_gap = 20;
% NV position pred plot
figure(104)
hold on
for i = 1:step_gap:Tsim
    plot(step_matrix(i, :), s_nv(i, :))
end
% actual position line
plot(X_nv(:, 1), 'b', 'LineWidth',1)
xlabel('Time step')
ylabel('NV Position [m]')
title('NV Position Prediction')

% NV velocity pred plot
figure(105)
hold on
for i = 1:step_gap:Tsim
    plot(step_matrix(i, :), v_nv(i, :))
end
% actual position line
plot(X_nv(:, 2), 'b', 'LineWidth',1)
xlabel('Time step')
ylabel('NV Velocity [m/s]')
title('NV Velocity Prediction')

% NV acceleration pred plot
figure(106)
hold on
for i = 1:step_gap:Tsim
    plot(step_matrix(i, :), a_nv(i, :))
end
% actual position line
plot(X_nv(:, 3), 'b', 'LineWidth',1)
xlabel('Time step')
ylabel('NV Acceleration [m/s^2]')
title('NV Acceleration Prediction')

%% Multi subject plots
subjects = ['ae1', 'am3', 'az4', 'jb4', 'jh1', 'ml3'];

figure(100)
set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5],'Ylim',[0 roadlength])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
xlabel('Lane number')
ylabel('Road length [m]')
roadlength = 100;
start = 10;
for i = 1:3:length(subjects)
    data = append(subjects(i), subjects(i+1), subjects(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    X_nv = log_data(start:fin, 15:18);
    X_obs = log_data(start:fin, 20);
    plot(X(:,4),X(:,1),'b','LineWidth',1)
    plot(X_nv(:,4),X_nv(:,1),'r','LineWidth',1)
    rectangle('Position',[0.5 (X_obs(1)-(5/2)) 0.5 5],'FaceColor',[1 0.8 0.8])
end