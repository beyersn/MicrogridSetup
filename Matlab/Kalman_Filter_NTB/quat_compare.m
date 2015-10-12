function [ ] = quat_compare(x, Actual_Attitude, UKF_Attitude)

% r = find(Attitude_Error==max(Attitude_Error(:)));

DCM1=quat2dcm(Actual_Attitude(x,:));
DCM2=quat2dcm(UKF_Attitude(x,:));

xunit=[1 0 0]';
yunit=[0 1 0]';
zunit=[0 0 1]';

q1x=DCM1*xunit;
q1y=DCM1*yunit;
q1z=DCM1*zunit;

q2x=DCM2*xunit;
q2y=DCM2*yunit;
q2z=DCM2*zunit;


figure(); hold('on'); grid('on');
xlabel('X-Body');
ylabel('Y-Body');
zlabel('Z-Body');
line_colors = [1 0 0; 0 0.5 0; 0 0 1];

%Actual Attitude Quaternian
plot3([0 q1x(1,:)],[0 q1x(2,:)], [0 q1x(3,:)], 'Color', line_colors(1,:))
plot3([0 q1y(1,:)],[0 q1y(2,:)], [0 q1y(3,:)], 'Color', line_colors(2,:))
plot3([0 q1z(1,:)],[0 q1z(2,:)], [0 q1z(3,:)], 'Color', line_colors(3,:))

%UKF Attitude Quaternian
plot3([0 q2x(1,:)],[0 q2x(2,:)], [0 q2x(3,:)], 'Color', line_colors(1,:), 'LineStyle', '--')
plot3([0 q2y(1,:)],[0 q2y(2,:)], [0 q2y(3,:)], 'Color', line_colors(2,:), 'LineStyle', '--')
plot3([0 q2z(1,:)],[0 q2z(2,:)], [0 q2z(3,:)], 'Color', line_colors(3,:), 'LineStyle', '--')

end

