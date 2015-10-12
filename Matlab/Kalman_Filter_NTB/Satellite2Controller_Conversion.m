% Brandon Jackson
% bajackso@mtu.edu
% 8 August 2013
% This function determiens the quaternion for the vector rotation between
% the satellite coordinate system and controller coordinate system.

%% Principal axis of inertia taken at the center of mass
Cx = [0.034, 0.025, 0.999];
Cy = [0.771, -0.636, -0.010];
Cz = [0.635, 0.771, -0.041];

Cxyz = [Cx; Cy; Cz];

Px = 1483543556.057;
Py = 3265113944.978;
Pz = 3371404672.819;

Pxyz = [Px 0 0; 0 Py 0; 0 0 Pz];

Sx = [1, 0, 0];
Sy = [0, 1, 0];
Sz = [0, 0, 1];

Sxyz = eye(3);

Origin = [0, 0, 0];

Lxx = 3305966706.380;	Lxy = -50539291.912;	Lxz = 63179402.319;
Lyx = -50539291.912;	Lyy = 3327170556.849;	Lyz = 48594457.548;
Lzx = 63179402.319;     Lzy = 48594457.548;     Lzz = 1486924910.625;

Lxyz = [Lxx, Lxy, Lxz; Lyx, Lyy, Lyz; Lzx, Lzy, Lzz];

R = Pxyz*inv(Lxyz);%#ok
q_s_c = dcm2quat(R);

%% Plot Axis
figure(); hold('on'); grid('on');
xlabel('X-Body');
ylabel('Y-Body');
zlabel('Z-Body');
line_colors = [1 0 0; 0 0.5 0; 0 0 1];

% Plot Satellite Coordinate System
plot3([Origin(1), Sx(1)], [Origin(2), Sx(2)], [Origin(3), Sx(3)],...
    'Color', line_colors(1,:), 'LineWidth', 2);
plot3([Origin(1), Sy(1)], [Origin(2), Sy(2)], [Origin(3), Sy(3)],...
    'Color', line_colors(2,:), 'LineWidth', 2);
plot3([Origin(1), Sz(1)], [Origin(2), Sz(2)], [Origin(3), Sz(3)],...
    'Color', line_colors(3,:), 'LineWidth', 2);

% Plot Controller Coordinate System,
plot3([Origin(1), Cx(1)], [Origin(2), Cx(2)], [Origin(3), Cx(3)],...
    'Color', line_colors(1,:), 'LineWidth', 2, 'LineStyle', '--');
plot3([Origin(1), Cy(1)], [Origin(2), Cy(2)], [Origin(3), Cy(3)],...
    'Color', line_colors(2,:), 'LineWidth', 2, 'LineStyle', '--');
plot3([Origin(1), Cz(1)], [Origin(2), Cz(2)], [Origin(3), Cz(3)],...
    'Color', line_colors(3,:), 'LineWidth', 2, 'LineStyle', '--');