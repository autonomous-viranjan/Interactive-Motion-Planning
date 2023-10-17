% Viranjan Bhattacharyya
% EMC2 Lab Clemson University

clear, clc, close all

%%
subject = 'jh1';
key = append(subject, '.txt');
log_data = load(key);
%%
start = 30;
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
roadlength = 100;
%%
figure(1)
set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5],'Ylim',[0 roadlength])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
plot(X_ref(:,4),X_ref(:,1),'-.k','LineWidth',2)
plot(X(:,4),X(:,1),'b','LineWidth',2)
plot(X_nv(:,4),X_nv(:,1),'r','LineWidth',2)
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
figure(333)
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
legend('$\alpha_{p}$','$\alpha_{a}$','Interpreter','Latex')
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
step_gap = 10;
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
%%
figure(111)
subplot(411)
plot(X(:,4),'b', 'LineWidth',1)
ylabel('Lane')
title('Ego lane position')
xlim([0 500])
subplot(412)
plot(alpha_a,'-.', 'LineWidth',2)
hold on
plot(alpha_v,'--', 'LineWidth',2)
lgd = legend('$\alpha_{a}$','$\alpha_{p}$','Interpreter','Latex');
lgd.FontSize = 12;
title('Imputation')
xlim([0 500])
subplot(413)
% NV acceleration pred plot
hold on
for i = 1:step_gap:Tsim
    plot(step_matrix(i, :), a_nv(i, :))
end
plot(X_nv(:, 3), 'r', 'LineWidth',1)
ylabel('a_{NV} [m/s^2]')
title('NV acceleration and predictions')
xlim([0 500])
subplot(414)
% NV velocity pred plot
hold on
for i = 1:step_gap:Tsim
    plot(step_matrix(i, :), v_nv(i, :))
end
% actual
plot(X_nv(:, 2), 'r', 'LineWidth',1)
xlabel('Time step')
ylabel('v_{NV} [m/s]')
title('NV velocity and predictions')
xlim([0 500])
%% Multi subject analysis
clear, clc, close all
%%
subjects_aimpc = ['ae1', 'am3', 'az4', 'jb4', 'jh1', 'ml3']; % 50 m truck
% subjects_aimpc = ['ae2', 'am2', 'az2', 'jb3', 'jh2', 'ml2']; % 60 m truck
% subjects_aimpc = ['ae3', 'am1', 'az1', 'jb1', 'jh3','ml1']; % 70 m truck

% Plots
roadlength = 100;
figure(100)
title('aiMPC')
set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5],'Ylim',[0 roadlength])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
xlabel('Lane number')
ylabel('Road length [m]')
start = 30;
for i = 1:3:length(subjects_aimpc)
    data = append(subjects_aimpc(i), subjects_aimpc(i+1), subjects_aimpc(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    X_nv = log_data(start:fin, 15:18);
    X_obs = log_data(start:fin, 20);
    plot(X(:,4),X(:,1),'b','LineWidth',1)
    plot(X_nv(:,4),X_nv(:,1),'r','LineWidth',1)
    rectangle('Position',[0.5 (X_obs(1)-(5/2)) 0.5 5],'FaceColor',[1 0.8 0.8])
end
% Velocity analysis
v_avg_all = [];
v_nv_avg_all = [];
for i = 1:3:length(subjects_aimpc)
    data = append(subjects_aimpc(i), subjects_aimpc(i+1), subjects_aimpc(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    v = X(:,2);
    v_avg = mean(v);
    v_avg_all = [v_avg_all; v_avg];
    X_nv = log_data(start:fin, 15:18);
    v_nv = X_nv(:,2);
    v_nv_avg = mean(v_nv);
    v_nv_avg_all = [v_nv_avg_all; v_nv_avg];
end

ego_avg_v = mean(v_avg_all)
nv_avg_v = mean(v_nv_avg_all)
%% Velocity plots
figure(150)
subplot(313)
hold on
for i = 1:3:length(subjects_aimpc)
    data = append(subjects_aimpc(i), subjects_aimpc(i+1), subjects_aimpc(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    v = X(:,2);
    X_nv = log_data(start:fin, 15:18);
    v_nv = X_nv(:,2);
    plot(v, 'b')
    plot(v_nv, 'r')
end
title('aiMPC')
xlabel('Time step')
ylabel('Velocity [m/s]')
xlim([0 500])
ylim([0 15])
%% Imputation
figure(300)
start = 30;
j = 1;
for i = 1:3:length(subjects_aimpc)
    data = append(subjects_aimpc(i), subjects_aimpc(i+1), subjects_aimpc(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    subplot(3,2,j)
    alpha_p = log_data(start:fin, 22);
    alpha_a = log_data(start:fin, 23);
    plot(alpha_a,'-.','LineWidth',2)
    hold on
    plot(alpha_p,'--','LineWidth',2)
    lg = legend('$\alpha_{a}$','$\alpha_{p}$','Interpreter','Latex');
    lg.FontSize = 14;
    title(['Subject ', num2str(j)])
    xlim([0 500]);
    xlabel('Time step')
    j = j+1;
end
sgtitle('Imputation')
%%
start = 30;
j = 1;
figure(500)
for i = 1:3:length(subjects_aimpc)
    data = append(subjects_aimpc(i), subjects_aimpc(i+1), subjects_aimpc(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    X_nv = log_data(start:fin, 15:18);
    X_obs = log_data(start:fin, 20);
    set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5])
    subplot(2,3,j)
    plot(X(:,4),X(:,1),'b','LineWidth',1)
    hold on
    plot(X_nv(:,4),X_nv(:,1),'r','LineWidth',1)
    title(['Subject ', num2str(j)])
    ylim([10 100])
    % rectangle('Position',[0.5 (X_obs(1)-(5/2)) 0.5 5],'FaceColor',[1 0.8 0.8])
    plot(0.75, X_obs(1), 'square', 'LineWidth',2, 'Color','r')
    set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5])
    j = j+1;
end