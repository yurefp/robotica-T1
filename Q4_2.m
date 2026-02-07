clear; clc; close all;

% Definição dos elos (DH clássico)
L(1) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', -pi/2);
L(2) = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', 0);

% Criação do robô
robot = SerialLink(L, 'name', 'DKP400');

% Transformação Base -> Frame 0
robot.base = transl(0.0, 0.0, 0.51) * troty(pi/2);

% Transformação Elo final -> Efetuador
robot.tool = transl(0, 0, 0.447) * trotz(-pi/2);

% Configuração das juntas
q = [1.479 0.3826];

T = robot.fkine(q);

% Plot
robot.plot(q);
robot.teach;
