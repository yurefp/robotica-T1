clear; clc; close all;

q_b = [-1.2764 0.6857 -1.7073 0.3506 0.7602 1.3329];
%q_b = [-0.2073 -2.9612 -2.0336 -2.1091 -1.3828 -3.7221];
%q_b = [-0.4974 -1.8194 0.5113 -1.3379 1.5663 -1.6302];
%q_b = [-2.4928 0.1397 2.1512 0.1646 -0.4733 3.7511];

q_m = [-0.4612 -1.4962];
%q_m = [-1.3348 -1.3143];
%q_m = [1.3745 -1.2494];
%q_m = [1.479 0.3826];

T01_b = transl(0.0, 0.0, 0.575) * trotz(-q_b(1));    % Transformação junta 0 -> junta 1 do braço
T12_b = transl(0.175, 0.0, 0.0) * troty(q_b(2));    % Transformação junta 1 -> junta 2 do braço
T23_b = transl(0.89, 0.0, 0.0) * troty(q_b(3));    % Transformação junta 2 -> junta 3 do braço
T34_b = transl(0.0, 0.0, 0.05) * trotx(-q_b(4));    % Transformação junta 3 -> junta 4 do braço
T45_b = transl(1.035, 0.0, 0.0) * troty(q_b(5));    % Transformação junta 4 -> junta 5 do braço
T56_b = transl(0.185, 0.0, 0.0) * (troty(pi/2)*trotz(-q_b(6)));    % Transformação junta 5 -> junta 6 do braço
T6t_b = transl(0.000361, 0.0, 0.461877) * troty(deg2rad(33.89));   % Transformação junta 6 -> efetuador do braço

Tabt = T01_b * T12_b * T23_b * T34_b * T45_b * T56_b * T6t_b;

T01_m = transl(0.0, 0.0, 0.51) * (trotz(pi)*trotx(-q_m(1)));    % Transformação junta 0 -> junta 1 da mesa
T12_m = transl(0.0, 0.0, 0.347) * trotz(q_m(2));    % Transformação junta 1 -> junta 2 da mesa
T2de_m = transl(0.0, 0.0, 0.1);   % Transformação junta 2 -> efetuador da mesa

Ttbde = T01_m * T12_m * T2de_m;

%Transformação base do braço -> base da mesa (junta 0 do braço -> junta 0 da mesa)
Tabtb = [-1 0 0 0.90;
        0 -1 0 0.00;
        0 0 1 -0.48;
        0 0 0 1];

%Transformação base do braço -> efetuador da mesa
Tabde = Tabtb * Ttbde;

%Transformação efetuador da mesa -> efetudor do braço
Tdet = Tabde\Tabt;

R = Tdet(1:3, 1:3);

% Cálculo RPY
pitch = asin(R(1,3));
yaw  = atan2(-R(2,3), R(3,3));
roll   = atan2(-R(1,2), R(1,1));

fprintf('%.4f %.4f %.4f %.4f %.4f %.4f %.4f', Tdet(1,4), Tdet(2,4), Tdet(3,4), rad2deg(roll), rad2deg(pitch), rad2deg(yaw));
