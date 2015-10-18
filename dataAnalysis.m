clear
close all
clc
format long

time_stamp = '03233820';

states_file_name = strcat('data/state/state_',time_stamp);
cmmd_file_name = strcat('data/transmitter/transmitter_',time_stamp);

states_file_name = strcat(states_file_name, '.txt');
cmmd_file_name = strcat(cmmd_file_name, '.txt');


states = importdata(states_file_name);
cmmd = importdata(cmmd_file_name);

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

for i = 1:length(states)
    
    state_time_tmp = (states(i,1) - states(1,1))/1e6;
    cmmd_time_tmp = (cmmd(i,1) - cmmd(1,1))/1e6;
    accel_z_tmp = -((states(i,4))/9.8*10);
    accel_y_tmp = states(i, 3)/9.8*10;
    thrust_cmmd_tmp = cmmd(i,4);
    
    yaw_vel_tmp = deg2rad(states(i, 10));
    yaw_cmmd_tmp = cmmd(i, 5);
    
    pitch_vel_tmp = deg2rad(states(i,9));
    pitch_angle_tmp = deg2rad(states(i,6));
    pitch_cmmd_tmp = cmmd(i, 2);
    
    roll_vel_tmp = deg2rad(states(i, 8));
    roll_angle_tmp = deg2rad(states(i, 5));
    roll_cmmd_tmp = cmmd(i, 3);
    
    if(state_time_tmp >= 2 & state_time_tmp <= 36)
        
        states_time = [states_time; state_time_tmp];
        cmmd_time = [cmmd_time; cmmd_time_tmp];
        accel_y = [accel_y; accel_y_tmp];
        accel_z = [accel_z; accel_z_tmp];
        thrust_cmmd = [thrust_cmmd; thrust_cmmd_tmp];
        
        yaw_cmmd = [yaw_cmmd; yaw_cmmd_tmp];
        yaw_vel = [yaw_vel; yaw_vel_tmp];
        
        pitch_vel = [pitch_vel; pitch_vel_tmp];
        pitch_angle = [pitch_angle; pitch_angle_tmp];
        pitch_cmmd = [pitch_cmmd; pitch_cmmd_tmp];
        
        roll_vel = [roll_vel; roll_vel_tmp];
        roll_angle = [roll_angle; roll_angle_tmp];
        roll_cmmd = [roll_cmmd; roll_cmmd_tmp];
        
    end
        
    
end

% states_time = (states(:,1) - states(1,1))/1e6;
% cmmd_time = (cmmd(:,1) - cmmd(1,1))/1e6;
% 
% accel_z = (states(:,4));
acc_z_filtered = medfilt1(accel_z, 10);

% thrust_cmmd = cmmd(:,4);

p = polyfit(acc_z_filtered,thrust_cmmd,1)
[cr, lgs] = xcorr(thrust_cmmd,acc_z_filtered,1)

figure(1)
plot(states_time, acc_z_filtered);
title('acceleration in z');
xlabel('Time');
ylabel('acceleration');

figure(2)
plot(cmmd_time, thrust_cmmd)
title('thrust command');
xlabel('Time')
ylabel('Acceleration')

figure(3)

plotyy(states_time, acc_z_filtered, cmmd_time, thrust_cmmd)
title('acceleration in z and thrust')

formatSpec = 'time_stamp: %s, coefficients: %f, %f\n';
fprintf(formatSpec, time_stamp, p(1), p(2))




yaw_vel_filtered = medfilt1(yaw_vel, 10);

figure(4)
plot(states_time, yaw_vel, 'g');
hold on;
plot(states_time, yaw_vel_filtered)

title('median filter')
figure(5)

% plot(states_time, yaw_cmmd)

plotyy(states_time, yaw_vel_filtered, states_time, yaw_cmmd);
p_yaw = polyfit(yaw_vel_filtered, yaw_cmmd,1)
title('yaw rate and yaw command versus time')

pitch_vel_filtered = medfilt1(pitch_vel, 10);
pitch_angle_filtered = medfilt1(pitch_angle, 10);
accel_y_filtered = medfilt1(accel_y, 10);

figure(6)
plotyy(states_time, pitch_vel_filtered, states_time, pitch_cmmd);
title('Pitch rate and pitch command versus time')

figure(7)
plotyy(states_time, pitch_angle_filtered, states_time, pitch_cmmd);
title('Pitch angle and pitch command versus time')

p_pitch = polyfit(pitch_angle_filtered, pitch_cmmd,1)

roll_vel_filtered = medfilt1(roll_vel, 10);
roll_angle_filtered = medfilt1(roll_angle, 10);

figure(8)
plotyy(states_time, roll_vel_filtered, states_time, roll_cmmd);
title('roll rate and roll command versus time')

figure(9)
plotyy(states_time, roll_angle_filtered, states_time, roll_cmmd);
title('roll angle and roll command versus time')
p_roll = polyfit(roll_angle_filtered, roll_cmmd,1)


figure(10)
plotyy(states_time, roll_angle_filtered, states_time, acc_z_filtered)
title('Roll angle and acceleration in Z versus time')

figure(11)
plotyy(states_time, accel_y_filtered, states_time, pitch_cmmd)
