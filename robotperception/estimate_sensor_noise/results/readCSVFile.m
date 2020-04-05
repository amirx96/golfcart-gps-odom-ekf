T = csvread('accelgps.csv');
acc_x = T(:, 1);
acc_y = T(:, 2);
acc_z = T(:, 3);
gps_x = T(:, 4);
gps_y = T(:, 5);

mean_acc_x = mean(acc_x)
mean_acc_y = mean(acc_y)
mean_acc_z = mean(acc_z)
mean_gps_x = mean(gps_x)
mean_gps_y = mean(gps_y)

mean_removed_accx = acc_x-mean(acc_x);
mean_removed_accy = acc_y-mean(acc_y);
mean_removed_accz = acc_z-mean(acc_z);
mean_removed_gpsx = gps_x-mean(gps_x);
mean_removed_gpsy = gps_y-mean(gps_y);

std_accx = std(acc_x);
std_accy = std(acc_y);
std_accz = std(acc_z);
std_gpsx = std(gps_x);
std_gpsy = std(gps_y);

subplot(511)
plot(mean_removed_accx);
hold on;
plot(mean_removed_accx+3*std_accx, 'r-')
hold on;
plot(mean_removed_accx-3*std_accx, 'r-')
hold off;
subplot(512)
plot(mean_removed_accy);
hold on;
plot(mean_removed_accy+3*std_accy, 'r-')
hold on;
plot(mean_removed_accy-3*std_accy, 'r-')
hold off;
subplot(513)
plot(mean_removed_accz);
hold on;
plot(mean_removed_accz+3*std_accz, 'r-')
hold on;
plot(mean_removed_accz-3*std_accz, 'r-')
hold off;
subplot(514)
plot(mean_removed_gpsx);
hold on;
plot(mean_removed_gpsx+3*std_gpsx, 'r-')
hold on;
plot(mean_removed_gpsx-3*std_gpsx, 'r-')
hold off;
subplot(515)
plot(mean_removed_gpsy);
hold on;
plot(mean_removed_gpsy+3*std_gpsy, 'r-')
hold on;
plot(mean_removed_gpsy-3*std_gpsy, 'r-')
hold off;

