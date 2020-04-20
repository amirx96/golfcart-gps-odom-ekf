%% Data Loading and Manipulation 

ekf_state = import_ekf('2_results.csv');
ins_state = import_ins('2-vectornav-ins.csv');
[ins_x, ins_y] = llh_to_xy(ins_state.lon, ins_state.lat, ins_state.alt);


%% Plotting Overall Results

figure(1)
plot(ekf_state.pos_x, ekf_state.pos_y, ins_x, ins_y)
legend('EKF State Estimate', 'INS Position Estimate')
xlabel('Position Easting [meters]')
ylabel('Position Northing [meters]')
axis square 

%% Plot Jumpyness of INS