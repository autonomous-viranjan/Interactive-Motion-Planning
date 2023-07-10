% Viranjan Bhattacharyya
% EMC2 Lab Clemson University

clear, clc, close all

% log_data = load("log_jb3.txt");
% log_data = load("log_vb.txt");
log_data = load("log_az2.txt");
%%
% start = 154;
% fin = 590;
% start = 160;
% fin = 777;
start = 355;
fin = 1014;
X = log_data(start:fin, 1:5);
X_ref = log_data(start:fin, 7:11);
Ua = log_data(start:fin, 12);
Ul = log_data(start:fin, 13);
X_nv = log_data(start:fin, 15:17);
X_obs = log_data(start:fin, 19);
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