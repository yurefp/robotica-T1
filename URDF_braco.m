clear; clc; close all;

q = [pi/2 -pi/4 pi/3 pi/6 pi/2 -pi/5];
%q = [0 0 0 0 0 0];

T01 = transl(0.0, 0.0, 0.575) * trotz(-q(1));    % Transformação junta 0 -> junta 1
T12 = transl(0.175, 0.0, 0.0) * troty(q(2));    % Transformação junta 1 -> junta 2
T23 = transl(0.89, 0.0, 0.0) * troty(q(3));    % Transformação junta 2 -> junta 3
T34 = transl(0.0, 0.0, 0.05) * trotx(-q(4));    % Transformação junta 3 -> junta 4
T45 = transl(1.035, 0.0, 0.0) * troty(q(5));    % Transformação junta 4 -> junta 5
T56 = transl(0.185, 0.0, 0.0) * (troty(pi/2)*trotz(-q(6)));    % Transformação junta 5 -> junta 6
T6e = transl(0.000361, 0.0, 0.461877) * troty(deg2rad(33.89));   % Transformação junta 6 -> efetuador

T0e = T01 * T12 * T23 * T34 * T45 * T56 * T6e;

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




