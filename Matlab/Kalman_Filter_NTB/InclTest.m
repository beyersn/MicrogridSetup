% Gyro Constants
% w_bias = [0.5 0.2 0.15];     % rad/s
w_bias = [0 0 0];     % rad/s
% gyro_noise = (0.35*pi/180)^2;          % rad^2/s^2
gyro_noise = 0;          % rad^2/s^2

% Magnetometer Constants
B_bias = [0 0 0];               % Tesla
B_noise = ((1*0.0052)/1E5)^2;                % Tesla^2

% Operating Enviroment
w_init_s = [0.03 0.02 0.01];	% rad/s
Time_Start = 0;                     % min after epoch
run_time = 1000;                   % sec
Ts = .1;                           % sec

% Initial 
q_s_c = [1 0 0 0];
q_i_s0 = [1 0 0 0];
Torque_s = [0 0 0];
I_c = [3.4 3.4 1.9];

% Orbit (TLE)
longstr1 = '1 25544U 98067A   13213.51579142  .00005418  00000-0  10235-3 0  3046';

figure();
hold('on');
grid('on');
xlabel('Time (s)');
ylabel('Error (deg)');

Decimation = 1;
Simulation_Time = Time_Start:(Decimation*Ts):(Time_Start+run_time);

idx = 1;
incl_array = 0:10:80;
Color_Array = hsv(length(incl_array));
for incl = incl_array;
    clearvars -global
    if incl == 0
        longstr2 = ['2 25544  ', num2str(incl), '0.0000 243.7352 0003674 255.5105 239.9716 15.50141065841738'];
    else
        longstr2 = ['2 25544  ', num2str(incl), '.0000 243.7352 0003674 255.5105 239.9716 15.50141065841738'];
    end
    if length(longstr2) ~= 69
        warndlg('Oh shit!')
        break;
    end

% Initialize the SGP4 propagator
SGP4_Setup(longstr1, longstr2)
[ECEF_Init, V_ECEF_Init] = sgp4(Time_Start);
ECEF_Init = ECEF_Init*1000;
V_ECEF_Init = V_ECEF_Init*1000;
LLA_Init = ecef2lla(ECEF_Init');
Lat = LLA_Init(1);
Long = LLA_Init(2);
Alt = LLA_Init(3)/1000;
B_Init_Inertial = wrldmagm(Alt, Lat, Long, 2013)*1E-9;   % Convert nT to T;
NED_Init = dcmecef2ned(Lat, Long)*ECEF_Init;

sim('UKF_10hz');

plot(Simulation_Time, Attitude_Error, 'Color', Color_Array(idx, :), 'DisplayName', num2str(incl));
idx = idx + 1;
legend('-DynamicLegend');
end