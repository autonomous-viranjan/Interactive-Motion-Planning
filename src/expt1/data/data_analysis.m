% Viranjan Bhattacharyya
% EMC2 Lab Clemson University

clear, clc, close all

log_data = load("jh3.txt");
%%
start = 150;
fin = length(log_data);
X = log_data(start:fin, 1:5);
X_ref = log_data(start:fin, 7:11);
Ua = log_data(start:fin, 12);
Ul = log_data(start:fin, 13);
X_nv = log_data(start:fin, 15:17);
X_obs = log_data(start:fin, 19);
X_nv_pred = log_data(start:fin, 21);
roadlength = 140;
%%
figure(1)
set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5],'Ylim',[0 roadlength])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
plot(X_ref(:,4),X_ref(:,1),'-.k')
plot(X(:,4),X(:,1),'b')
plot(X_nv(:,3),X_nv(:,1),'r')
rectangle('Position',[0.5 (X_obs(1)-5) 0.5 5],'FaceColor',[0.9 0.8 0.8])
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
l_nv = X_nv(:,3)';
s_nv = X_nv(:,1)';
surface([l_nv;l_nv],[s_nv;s_nv],[z;z],[col;col],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
rectangle('Position',[0.5 (X_obs(1)-5) 0.5 5],'FaceColor',[0.9 0.9 0.9])
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
%%
figure(4)
plot(Ua, 'g')
hold on
plot(X(:,3))
plot(X_ref(:,3))
ylim([-2 2])
ylabel('m/s^2')
legend('Commanded acceleration','Ego acceleration','Reference acceleration')
%%
figure(5)
plot(X(:,2))
hold on
plot(X_nv(:,2))
ylabel('m/s')
legend('Ego', 'NV')
title('Speed')
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
plot(X_nv(:,3))
legend('Ego','NV')
ylabel('Lateral Position [m]')
xlabel('Time step')
%%
figure(8)
set (gca,'DataAspectRatio',[1 60 1],'Xdir','reverse','Xlim',[1.5 2.5])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
plot(X_nv(:,3),X_nv(:,1),'Color',[0.8 0.1 0.5],'LineWidth',2)
plot(2*ones(length(X_nv_pred)), X_nv_pred(:,1),'.r')
xlabel('Lane number')
ylabel('Road length [m]')
legend('','Actual','Predicted')
title('NV actual and predicted trajectory')

%% Multi subject
clear, clc, close all
%%
subjects_base = ['ae3', 'am3', 'az3', 'jb3', 'jh3', 'ml3']; % 50 m truck
% subjects_base = ['ae2', 'am2', 'az2', 'jb2', 'jh2', 'ml2']; % 60 m truck
% subjects_base = ['ae1', 'am1', 'az1', 'jb3', 'jh1','ml1']; % 70 m truck

% Plots
roadlength = 100;
figure(100)
title('Baseline MPC')
set (gca,'DataAspectRatio',[1 15 1],'Xdir','reverse','Xlim',[0.5 2.5],'Ylim',[0 roadlength])
hold on 
plot([1.5 1.5],[0 roadlength],'--','LineWidth',2,'Color',[0.5 0.5 0.5]) % lane marking
xlabel('Lane number')
ylabel('Road length [m]')

for i = 1:3:length(subjects_base)
    data = append(subjects_base(i), subjects_base(i+1), subjects_base(i+2), '.txt');
    log_data = load(data);
    if data(2) == 'e'
        start = 110;
    else
        start = 30;
    end
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    X_nv = log_data(start:fin, 15:17);
    X_obs = log_data(start:fin, 19);
    rectangle('Position',[0.5 (X_obs(1)-(5/2)) 0.5 5],'FaceColor',[0.9 0.8 0.8])
    plot(X(:,4),X(:,1),'b')
    plot(X_nv(:,3),X_nv(:,1),'r')
end
% Velocity analysis
v_avg_all = [];
v_nv_avg_all = [];
for i = 1:3:length(subjects_base)
    data = append(subjects_base(i), subjects_base(i+1), subjects_base(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    if data(2) == 'e'
        start = 110;
    else
        start = 30;
    end
    X = log_data(start:fin, 1:5);
    v = X(:,2);
    v_avg = mean(v);
    v_avg_all = [v_avg_all; v_avg];
    X_nv = log_data(start:fin, 15:17);
    v_nv = X_nv(:,2);
    v_nv_avg = mean(v_nv);
    v_nv_avg_all = [v_nv_avg_all; v_nv_avg];
end

ego_avg_v = mean(v_avg_all)
nv_avg_v = mean(v_nv_avg_all)
%% Velocity plots
figure(150)
subplot(311)
hold on
for i = 1:3:length(subjects_base)
    data = append(subjects_base(i), subjects_base(i+1), subjects_base(i+2), '.txt');
    log_data = load(data);
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    v = X(:,2);
    X_nv = log_data(start:fin, 15:17);
    v_nv = X_nv(:,2);
    plot(v, 'b')
    plot(v_nv, 'r')
end
title('Baseline MPC')
xlabel('Time step')
ylabel('Velocity [m/s]')
xlim([0 500])
ylim([0 15])