clear; clc; close all;

% Definição dos elos (DH clássico)
% Elos da mesa
L(1) = Link('d', -0.447, 'a', 0, 'alpha', pi/2, 'offset', -pi/2);
L(2) = Link('d', 0.9, 'a', 0, 'alpha', pi/2, 'offset', 0);
% Elos do braço
L(3) = Link('d', -0.545, 'a', 0.175, 'alpha', pi/2, 'offset', -pi/2);
L(4) = Link('d', 0, 'a', 0.89, 'alpha', 0, 'offset', 0);
L(5) = Link('d', 0, 'a', 0.05, 'alpha', pi/2, 'offset', -pi/2);
L(6) = Link('d', -1.035, 'a', 0, 'alpha', -pi/2, 'offset', 0);
L(7) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0);
L(8) = Link('d', -0.646339, 'a', 0, 'alpha', -deg2rad(33.89), 'offset', -pi/2);

% Criação do robô
robot = SerialLink(L, 'name', 'KR30-DKP400');

% Transformação Base -> Frame 0
robot.base = transl(0.0, 0.0, 0.0) * trotx(0);

% Transformação Elo final -> Efetuador
robot.tool = transl(0, 0, -0.000648) * (trotx(-pi)*trotz(pi/2));

% Configuração das juntas
q1 = [-0.4612 -1.4962 -1.2764 0.6857 -1.7073 0.3506 0.7602 1.3329];
q2 = [-1.3348 -1.3143 -0.2073 -2.9612 -2.0336 -2.1091 -1.3828 -3.7221];
q3 = [1.3745 -1.2494 -0.4974 -1.8194 0.5113 -1.3379 1.5663 -1.6302];
q4 = [1.479 0.3826 -2.4928 0.1397 2.1512 0.1646 -0.4733 3.7511];

T = robot.fkine(q1);

xyz = T.t';
disp(xyz)

% Plot
robot.plot(q1);
robot.teach(q1);
