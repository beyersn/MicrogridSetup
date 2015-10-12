%% Set Simulation Constants
global satrec gravc
clear satrec gravc
clear

tm = datestr(now,'yyyymmdd-HHMMss-');
note = inputdlg('Enter descriptive word for this simulation');

odir = strcat('output/', tm, note{1});

mkdir('output');
mkdir(odir);
odir = strcat(odir, '/');

% Gyro Constants
w_bias = 0*[0.5 0.2 0.15];     % rad/s
%gyro_noise = (0.35*pi/180)^2;  % rad^2/s^2
gyro_noise = 0;          % rad^2/s^2

% Magnetometer Constants
B_bias = [0 0 0];               % Tesla
%B_noise = ((1*0.0052)/1E5)^2;                % Tesla^2
B_noise = 0;

% Operating Enviroment
rand_w_x=rand*.2;
rand_w_y=rand*.2;
rand_w_z=rand*.2;
w_init_s = [rand_w_x rand_w_y rand_w_z];	% rad/s
w_init_s = [.1 0 0]*pi/180;
% w_init_s = [.046 .17 .039];	% rad/s
Mean_Velocity=norm(w_init_s);
Time_Start = 0;          % min after epoch
run_time = 1000;                   % sec
Ts = .1;                           % sec

% Initial 
q_s_c = [1 0 0 0];
phi = 10*pi/180;
v = rand(1,3);
v = v/norm(v);
init_attitude = [cos(phi/2) sin(phi/2) 0 0];
%init_attitude = [1 0 0 0];
B_ECI_phase = [10 180 270]*pi/180;
dB_ECI = [.01 .01 .01]*pi/180;
q_i_s0 = init_attitude;
Torque_s = [0 0 0];
I_c = [3.4 3.4 1.9];

%Select the TLE
[longstr1,longstr2, rand_TLE]=GetTLE();

% Initialize the SGP4 propagator
SGP4_Setup(longstr1, longstr2)
ECEF_Init = sgp4(Time_Start)*1000;
LLA_Init = ecef2lla(ECEF_Init');
Lat = LLA_Init(1);
Long = LLA_Init(2);
Alt = LLA_Init(3)/1000;
B_Init_Inertial = wrldmagm(Alt, Lat, Long, 2013)*1E-9;   % Convert nT to T;
NED_Init = dcmecef2ned(Lat, Long)*ECEF_Init;

Decimation = 1;
N = run_time/Ts+1;
Attitude_sensor = zeros(N, 4);
w_sensor=zeros(N, 3);
w_bias_sensor=zeros(N, 3);
Kalman_Gain=zeros(N, 9, 9);
covariance=zeros(N,9,9);
z_kkm1_control=zeros(N,9);
Attitude_Error = zeros(N,1);
w_hist=zeros(N, 3);

q_hist=zeros(N,4);

B_ECI_hist=zeros(N,3);
B_sat_hist=zeros(N,3);
Bdot_hist = zeros(N,3);

q = init_attitude;


%For initialization of history
B_ECI = sin(B_ECI_phase);

%% Run The Simulation;
for i=1:N
    %Update B in ECI
    B_ECIm1 = B_ECI;
	
	%B_ECI = [1 1 1];
	B_ECI = sin(i*dB_ECI+B_ECI_phase);
    B_ECI = B_ECI/norm(B_ECI);
	
	Bdot_ECI = (B_ECI - B_ECIm1)/Ts;
	
    %Update actual attitude
    qdot = 0.5*quatmultiply(q, [0, w_init_s]);
    q = q+qdot.*Ts;
    q_hist(i,:)=q;
    
    %Update B in sat frame
    B_sat = quatrotate(quatconj(q), B_ECI)+normrnd(0, B_noise.^0.5);
    
    % Make w measurement
    w = normrnd(w_init_s, gyro_noise.^0.5);
    
    [A, W, B, K, P, Z] = UKF_quaternion_withBias(B_ECI, Bdot_ECI, B_sat, w, Torque_s, Ts, i==1);
    Attitude_sensor(i,:)=A;
    w_sensor(i,:) = W;
    w_bias_sensor(i,:) = B;
    Kalman_Gain(i, :, :) = K;
    covariance(i,:,:) = P;
    z_kkm1_control(i,:) = Z;
    dq = quatmultiply(q, quatconj(A));
    Attitude_Error(i,:) = 2*acos(dq(1))*180/pi;
    B_ECI_hist(i,:) = B_ECI;
    B_sat_hist(i,:) = B_sat;
	Bdot_hist(i,:) = Bdot_ECI;
    % Update simulation
    
  
end

%% Plot the Results
Simulation_Time = Time_Start:(Decimation*Ts):(Time_Start+run_time);

% Line_colors = [1 0 0; 0 0.5 0; 0 0 1; 0.7 0 0.7];
% Line_styles = {'-', '--', ':'};
% Line_width = [1, 2];
% 
% % Plot Bias
% figure();
% hold('on')
% grid('on');
% for idx = 1:3;
% plot(Simulation_Time, UKF_Gyro_Bias(:,idx)*180/pi,...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:));
% end
% xlabel('Time (seconds)');
% ylabel('Gyro Bias (deg/s)');
% legend('X', 'Y', 'Z');
% PrettyUpPlot
% 
% % Plot Angular rate
% figure();
% hold('on');
% grid('on');
% for idx = 1:3
% plot(Simulation_Time, UKF_Angular_Velocity(:,idx)*180/pi,...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(1));
% plot(Simulation_Time, Actual_Angular_Velocity(:,idx)*180/pi,...
%     'LineStyle', Line_styles{2}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(2));
% end
% xlabel('Time (seconds)');
% ylabel('Angular Velocity (deg/s)');
% 
% % Fake legend
% ax = axes; hold('on');
% plot(rand(1,2), 'color', [1 0 0], 'visible', 'off');
% plot(rand(1,2), 'color', [0 0.5 0], 'visible', 'off');
% plot(rand(1,2), 'color', [0 0 1], 'visible', 'off');
% set(ax, 'visible', 'off');
% legend('X', 'Y', 'Z');
% PrettyUpPlot
% 
% % Plot Angular Velocity Error
% figure();
% hold('on');
% grid('on');
% for idx = 1:3
% plot(Simulation_Time, (Actual_Angular_Velocity(:,idx) - UKF_Angular_Velocity(:,idx))*180/pi,...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(1));
% end
% xlabel('Time (seconds)');
% ylabel('Angular Velocity Error (deg/s)');
% legend('X', 'Y', 'Z');
% PrettyUpPlot
% ylim([-3 3])
% %% Plot Attitude
% figure();
% hold('on');
% grid('on');
% for idx = 1:4;
% plot(Simulation_Time, UKF_Attitude(:,idx),...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(1));
% plot(Simulation_Time, Actual_Attitude(:,idx),...
%     'LineStyle', Line_styles{2}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(2));
% end
% xlabel('Time (seconds)');
% xlabel('Quaternion Attitude');
% PrettyUpPlot
% 
% % Plot Attitude Error
% figure();
% hold('on');
% grid('on');
% for idx = 1:3;
% plot(Simulation_Time, Attitude_Error(:,idx),...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:));
% end
% xlabel('Time (seconds)');
% ylabel('Quaternion Attitude Error');
% legend('Z', 'Y', 'X');
% PrettyUpPlot

A=mat2str(w_init_s,2);
B=num2str(Mean_Velocity);
C=int2str(rand_TLE);
D=mat2str(q_i_s0,2);
E=int2str(i);

% Plot Attitude Error
h=figure();
hold('on');
grid('on');
% error_mag = sqrt(sum(Attitude_Error.^2,2));
error_mag=Attitude_Error;
plot(Simulation_Time, error_mag, 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Euler Angle Error Magnitude (degrees)');
legend('Error Magnitude');
PrettyUpPlot;
saveas(h,[odir 'dq.png']);

h=figure();
hold('on');
grid('on');
plot(Simulation_Time(1:9999), B_sat_hist(1:9999,:) - z_kkm1_control(2:10000,1:3), 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('B Field Prediction Error (arb)');
legend('dBx', 'dBy', 'dBz');
PrettyUpPlot;

saveas(h,[odir 'dB.png']);
save([odir 'workspace.mat'], '-v7.3');

close(h);
