clear; clc; close all;

q = [1.479 0.3826];
%q = [0 0];

T01 = transl(0.0, 0.0, 0.51) * (trotz(pi)*trotx(-q(1)));    % Transformação junta 0 -> junta 1
T12 = transl(0.0, 0.0, 0.347) * trotz(q(2));    % Transformação junta 1 -> junta 2
T2e = transl(0.0, 0.0, 0.1);   % Transformação junta 2 -> efetuador

T0e = T01 * T12 * T2e;

% Extrai matriz de rotação
R = T0e(1:3, 1:3);

% Cálculo RPY
pitch = asin(R(1,3));
yaw  = atan2(-R(2,3), R(3,3));
roll   = atan2(-R(1,2), R(1,1));

fprintf('x = %.4f\n', T0e(1,4));
fprintf('y = %.4f\n', T0e(2,4));
fprintf('z = %.4f\n', T0e(3,4));
fprintf('roll = %.4f\n', rad2deg(roll));
fprintf('pitch = %.4f\n', rad2deg(pitch));
fprintf('yaw = %.4f\n', rad2deg(yaw));

fprintf("%.4f %.4f %.4f %.4f %.4f %.4f", T0e(1,4), T0e(2,4), T0e(3,4), rad2deg(roll), rad2deg(pitch), rad2deg(yaw))


