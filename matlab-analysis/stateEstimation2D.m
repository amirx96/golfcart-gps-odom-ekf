%% add deg2utm to path
addpath('deg2utm/');

%% Read data
clear;
clc;
close all;
bagname = 'bag3';
acc_gyro = csvread(strcat(bagname, '/accel_gyro.csv'));
gps = csvread(strcat(bagname, '/gps.csv'));
orientation = csvread(strcat(bagname, '/orentation.csv'));
velocity = csvread(strcat(bagname, '/velocity.csv'));
%% Mapping IMU and GPS time stamps
% LLB = Lat, Long, Bearing
initial_LLB = [gps(1, 2), gps(1, 3), orientation(1, 4)];
initial_bearing = orientation(1, 4)*pi/180;
[initial_easting, initial_northing, ~] = deg2utm(initial_LLB(1),... 
                                                 initial_LLB(2));

ENO = [];
for i=1:length(gps)
    [e, n, ~] = deg2utm(gps(i, 2), gps(i, 3));
    o = orientation(i, 4)*pi/180;
    time_stamp = gps(i, 1);
    eno = [e, n, o];
    ENO = [ENO; eno];
end

figure(1)
plot(ENO(1, 1), ENO(1, 2), 'bd');
hold on;
plot(ENO(:, 1), ENO(:, 2));
hold on;


ts_gps = gps(:, 1);
ts_i = 0;

no_of_ts = length(acc_gyro);
time_stamp_map = [];
for i=1:no_of_ts
    ts = acc_gyro(i, 1);
    row_idx_acc = find(ts_gps < ts);
    time_stamp_map = [time_stamp_map; i, row_idx_acc(end)];
end

%% State Estimation
% State Vector is defined as X, Y, Vx, Vy, Yaw
% in the Navigation frame

stateVector = zeros(5, 1);
stateVector(1) = initial_northing;
stateVector(2) = initial_easting;
stateVector(5) = initial_bearing;
bearing = initial_bearing;
dt_imu = 0.02;

P = zeros(5, 5);
Q = diag([10, 10, 10]);
R = diag([0.5, 0.5, 1]);

H = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 0 0 1];
 
gps_idx = 2;
ENO_estimated = [initial_easting, initial_northing, bearing];
VEN_estimated = [];
t0 = gps(1,1);
for i = 1:no_of_ts
    dt_imu = acc_gyro(i, 1) - t0;
    Rotn = [cos(bearing) -sin(bearing);
            sin(bearing)  cos(bearing)];
    u = [acc_gyro(i, 2); acc_gyro(i, 3); acc_gyro(i, end)];
    
    A = [1, 0, dt_imu,      0, 0;
         0, 1,      0, dt_imu, 0;
         0, 0,      1,      0, 0;
         0, 0,      0,      1, 0;
         0, 0,      0,      0, 1];
    
    B = [Rotn*(dt_imu^2)/2, zeros(2, 1);
         Rotn*dt_imu, zeros(2, 1);
         0, 0, dt_imu];
     
    stateVector = A*stateVector+B*u;
    P = A*P*A' + B*Q*B';
    
    bearing = stateVector(5);
    sb = sin(bearing);
    cb = cos(bearing);
    bearing = atan2(sb, cb);
    stateVector(5) = bearing;
    
    if mod(i, 10) == 1 && i ~=1
        if gps_idx <= length(ENO)
            z = [ENO(gps_idx, 2); 
                 ENO(gps_idx, 1); 
                 ENO(gps_idx, 3)];
            innovation = z - H*stateVector;
            S = H*P*H' + R;
            K = P*H'*inv(S);
            stateVector = stateVector + K*innovation;
            P = P - K*H*P;
            P = 0.5 * (P + P');
            gps_idx = gps_idx+1;
            bearing = stateVector(5);
            sb = sin(bearing); 
            cb = cos(bearing);
            bearing = atan2(sb, cb);
            stateVector(5) = bearing;
        end
    end
    ENO_estimated = [ENO_estimated; 
                    stateVector(2), stateVector(1), stateVector(5)];
    VEN_estimated = [VEN_estimated; [stateVector(4), stateVector(3)]];
    t0 = acc_gyro(i, 1);
end
figure(1)
plot(ENO_estimated(1, 1), ENO_estimated(1, 2), 'r*');
hold on;
plot(ENO_estimated(:, 1), ENO_estimated(:, 2));
hold off;
axis square;
xlabel('Eastings (Y)');
ylabel('Northings (X)');
grid;

%%
figure(2)
subplot(311)
plot(gps(:, 1), ENO(:, 1), 'r');
hold on;
plot([gps(1,1);acc_gyro(:, 1)], ENO_estimated(:, 1), 'g');
grid;
ylabel('Easting');
hold off;
subplot(312)
plot(gps(:, 1), ENO(:, 2), 'r');
hold on;
plot([gps(1,1);acc_gyro(:, 1)], ENO_estimated(:, 2), 'g');
grid;
ylabel('Northing');
hold off;
subplot(313)
plot(gps(:, 1), ENO(:, 3), 'r');
hold on;
plot([gps(1,1);acc_gyro(:, 1)], ENO_estimated(:, 3), 'g');
grid;
ylabel('bearing');
hold off;

figure(3)
subplot(211)
plot(gps(:, 1), velocity(:, 3), 'r');
hold on;
plot([acc_gyro(:, 1)], VEN_estimated(:, 1), 'g');
grid;
ylabel('Ve');
hold off;

subplot(212)
plot(gps(:, 1), velocity(:, 2), 'r');
hold on;
plot([acc_gyro(:, 1)], VEN_estimated(:, 2), 'g');
grid;
ylabel('Ve');
hold off;

