clear
close all
clc
format long

time_stamp = '09115921';

states_file_name = strcat('data/state/state_',time_stamp);
% cmmd_file_name = strcat('data/transmitter/transmitter_',time_stamp);

states_file_name = strcat(states_file_name, '.txt');
% cmmd_file_name = strcat(cmmd_file_name, '.txt');


states = importdata(states_file_name);
% cmmd = importdata(cmmd_file_name);


states_time = [];
cmmd_time = [];
accel_y = [];
accel_z = [];
thrust_cmmd = [];

yaw_vel = [];
yaw_cmmd = [];

roll_vel = [];
roll_angle = [];
roll_cmmd = [];

pitch_vel = [];
pitch_angle = [];
pitch_cmmd = [];

rpm1 = [];
rpm2 = [];
rpm3 = [];
rpm4 = [];


for i = 1:length(states)
    
    state_time_tmp = (states(i,1) - states(1,1))/1e6;
%     cmmd_time_tmp = (cmmd(i,1) - cmmd(1,1))/1e6;
    accel_z_tmp = -((states(i,4))/9.8*10);
    accel_y_tmp = states(i, 3)/9.8*10;
%     thrust_cmmd_tmp = cmmd(i,4);
    
    rpm1_tmp  = states(i,11);
    rpm2_tmp  = states(i,12);
    rpm3_tmp  = states(i,13);
    rpm4_tmp  = states(i,14);    
    
    
    yaw_vel_tmp = deg2rad(states(i, 10));
%     yaw_cmmd_tmp = cmmd(i, 5);
    
    pitch_vel_tmp = deg2rad(states(i,9));
    pitch_angle_tmp = deg2rad(states(i,6));
%     pitch_cmmd_tmp = cmmd(i, 2);
    
    roll_vel_tmp = deg2rad(states(i, 8));
    roll_angle_tmp = deg2rad(states(i, 5));
%     roll_cmmd_tmp = cmmd(i, 3);
    
    if(state_time_tmp >= 0 & state_time_tmp <= 1000)
        
        states_time = [states_time; state_time_tmp];
%         cmmd_time = [cmmd_time; cmmd_time_tmp];
        accel_y = [accel_y; accel_y_tmp];
        accel_z = [accel_z; accel_z_tmp];
%         thrust_cmmd = [thrust_cmmd; thrust_cmmd_tmp];
        
%         yaw_cmmd = [yaw_cmmd; yaw_cmmd_tmp];
        yaw_vel = [yaw_vel; yaw_vel_tmp];
        
        pitch_vel = [pitch_vel; pitch_vel_tmp];
        pitch_angle = [pitch_angle; pitch_angle_tmp];
%         pitch_cmmd = [pitch_cmmd; pitch_cmmd_tmp];
        
        roll_vel = [roll_vel; roll_vel_tmp];
        roll_angle = [roll_angle; roll_angle_tmp];
%         roll_cmmd = [roll_cmmd; roll_cmmd_tmp];
        
        rpm1 = [rpm1; rpm1_tmp];
        rpm2 = [rpm2; rpm2_tmp];
        rpm3 = [rpm3; rpm3_tmp];
        rpm4 = [rpm4; rpm4_tmp];
    end
        
    
end


acc_z_filtered = medfilt1(accel_z, 10);
pitch_vel_filtered = medfilt1(pitch_vel, 10);
rpm1_filtered = medfilt1(rpm1, 10);
rpm2_filtered = medfilt1(rpm2, 10);
rpm3_filtered = medfilt1(rpm3, 10);
rpm4_filtered = medfilt1(rpm4, 10);


% figure(1)
% plotyy(states_time, rpm1_filtered, states_time, acc_z_filtered)
% figure(2)
% plotyy(states_time, rpm2, states_time, acc_z_filtered)
% figure(3)
% plotyy(states_time, rpm3, states_time, acc_z_filtered)
% figure(4)
% plotyy(states_time, rpm4, states_time, acc_z_filtered)

figure(1)
plotyy(states_time, rpm1_filtered, states_time, pitch_vel_filtered)
figure(2)
plotyy(states_time, rpm2_filtered, states_time, pitch_vel_filtered)
figure(3)
plotyy(states_time, rpm3_filtered, states_time, pitch_vel_filtered)
figure(4)
plotyy(states_time, rpm4_filtered, states_time, pitch_vel_filtered)