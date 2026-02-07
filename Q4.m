% Definição dos elos (DH clássico)
L(1) = Link('d', -0.575, 'a', 0.175, 'alpha', pi/2, 'offset', 0);
L(2) = Link('d', 0, 'a', 0.89, 'alpha', 0, 'offset', 0);
L(3) = Link('d', 0, 'a', 0.05, 'alpha', pi/2, 'offset', -pi/2);
L(4) = Link('d', -1.035, 'a', 0, 'alpha', -pi/2, 'offset', 0);
L(5) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0);
L(6) = Link('d', -0.646339, 'a', 0, 'alpha', -deg2rad(33.89), 'offset', -pi/2);

% Criação do robô
robot = SerialLink(L, 'name', 'Robo3DOF');

% Transformação Base -> Frame 0
robot.base = transl(0.0, 0.0, 0.0) * trotx(pi);

% Transformação Elo final -> Efetuador
robot.tool = transl(0, 0, -0.000648) * (trotx(-pi)*trotz(pi/2));

% Configuração das juntas
q = [0 0 0 0 0 0];

T = robot.fkine(q);

% Plot
robot.plot(q);
robot.teach;
