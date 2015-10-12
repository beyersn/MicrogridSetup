% Author: Brandon Jackson
% Contact: bajackso@mtu.edu
% Date: 26 June 2013
%
% Description: This function performs an analysis on the solar density
% above a fixed location of the Earth for the course of 1 day and 4 years
% to demonstrate that the atmospheric drag remains relatively constant
% during a given time period. This function does not account for f107
% models.

%% Density analysis over the course of a day
altitude = 800000;  % m
latitude = 20;      % degrees
longitude = 0;      % degrees
year = 2013;
dayOfYear = 200;

t_min = 0:15:(24*60);
UTseconds = t_min*60;
total_rho = zeros(size(t_min));
for i = 1:length(UTseconds)
    localApparentSolartime = UTseconds(i)/3600 + longitude/15;
    [T rho] = atmosnrlmsise00(altitude, latitude, longitude, year,...
        dayOfYear, UTseconds(i), localApparentSolartime, 'None');
    total_rho(i) = rho(6);
end

figure;
hold('on');
grid('on');
plot(t_min/60, total_rho, 'LineWidth', 2);
xlabel('Time (hours)');
ylabel('Total Density (kg/m^3)');
xlim([0 24]);
hold('off');

%% Density analysis over the course of a two years
altitude = 800000;  % m
latitude = 0;      % degrees
longitude = 0;      % degrees
eval_years = 1;
total_rho = zeros([eval_years*365,1]);
i = 1;
t_min = 0:15:(24*60);
UTseconds = t_min*60;
daily_rho = length(zeros(length(t_min)));
for year = 2013:(2013+eval_years)
    for dayOfYear = 1:365
        t_min = 0:15:(24*60);
        UTseconds = t_min*60;
        for j = 1:length(UTseconds)
            localApparentSolartime = UTseconds(j)/3600 + longitude/15;
            [T rho] = atmosnrlmsise00(altitude, latitude, longitude, year,...
            dayOfYear, UTseconds(j), localApparentSolartime, 'None');
            daily_rho(j) = rho(6);
        end
        max_total_rho(i) = max(daily_rho);
        avg_total_rho(i) = mean(daily_rho);
        i=i+1;
    end
end

figure;
hold('on');
grid('on');
plot((0:length(max_total_rho)-1)/365, max_total_rho, 'Color', [ 0 0 1],...
    'DisplayName', 'Daily Maximum', 'LineWidth', 2);
plot((0:length(avg_total_rho)-1)/365, avg_total_rho, 'Color', [ 1 0 0],...
    'DisplayName', 'Daily Average', 'LineWidth', 2);
xlabel('Time (years)');
ylabel('Total Density (kg/m^3)');
xlim([0 eval_years]);
legend('-DynamicLegend');
hold('off');