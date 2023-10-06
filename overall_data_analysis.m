clear, clc, close all;

const_vel = '/home/emc2desktop/vb/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning/src/expt1/data/';
joint_mpc = '/home/emc2desktop/vb/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning/src/expt2/data/';
aimpc1 = '/home/emc2desktop/vb/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning/src/expt4/data/';
aimpc2 = '/home/emc2desktop/vb/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning/src/expt5/data/';

expt = const_vel;
subject = 'jh1.txt';

data = append(expt, subject);
log_data = load(data);

if expt == const_vel
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
    roadlength = 100;
    %%
    figure(1)
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
            'linew',4);
    rectangle('Position',[0.5 (X_obs(1)-(5/2)) 0.5 5],'FaceColor',[0.9 0.8 0.8])
    xlabel('Lane number')
    ylabel('Road length [m]')
    text(X(1,4),X(1,1),'Ego','Color','b')
    text(X_nv(1,3),X_nv(1,1),'NV','Color','r')
    title('Trajectory')

elseif expt == joint_mpc
    start = 150;
    fin = length(log_data);
    X = log_data(start:fin, 1:5);
    X_ref = log_data(start:fin, 7:11);
    Ua = log_data(start:fin, 12);
    Ul = log_data(start:fin, 13);
    X_nv = log_data(start:fin, 15:18);
    X_obs = log_data(start:fin, 20);
    X_nv_pred = log_data(start:fin, 22:24);
    roadlength = 100;
    
    HZ = 20;
    
    T_nv = start/HZ:1/HZ:fin/HZ;
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
end