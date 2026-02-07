clear; clc; close all;

% Definição dos elos da mesa (DH Standard)
Lt(1) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', -pi/2);
Lt(2) = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', 0);

% Criação da mesa
DKP400 = SerialLink(Lt, 'name', 'DKP400');

% Transformação Base -> Frame 0
DKP400.base = transl(0.0, 0.0, 0.51) * troty(pi/2);

% Transformação Elo final -> Efetuador
DKP400.tool = transl(0, 0, 0.447) * trotz(-pi/2);

% Configuração das juntas da mesa
theta_t = [pi/4 0];

Ttb_de = DKP400.fkine(theta_t);

% Transformação Base do Braço -> Base da Mesa
Tat = transl(0.9, 0, -0.48) * trotz(pi);
Tat = SE3(Tat);

% Transformação Base do Braço -> Efetuador da Mesa
Tade = Tat * Ttb_de;

% Transformação Efetuador da Mesa -> Base do Braço
Rade = Tade.R;
pade = Tade.t;

Tade_inv = [Rade.'  -Rade.'*pade;
         0 0 0  1];

% Definição dos elos do braco (DH clássico)
La(1) = Link('d', -0.575, 'a', 0.175, 'alpha', pi/2, 'offset', 0);
La(2) = Link('d', 0, 'a', 0.89, 'alpha', 0, 'offset', 0);
La(3) = Link('d', 0, 'a', 0.05, 'alpha', pi/2, 'offset', -pi/2);
La(4) = Link('d', -1.035, 'a', 0, 'alpha', -pi/2, 'offset', 0);
La(5) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0);
La(6) = Link('d', -0.646339, 'a', 0, 'alpha', -deg2rad(33.89), 'offset', -pi/2);

% Criação do braço
KR30 = SerialLink(La, 'name', 'KR30');

% Transformação Efetuador da Mesa -> Frame 0
KR30.base = Tade_inv * (transl(0.0, 0.0, 0.0) * trotx(pi));

% Transformação Elo final -> Efetuador
KR30.tool = transl(0, 0, -0.000648) * (trotx(-pi)*trotz(pi/2));

%=======================================================================
% Cinemática inversa
%=======================================================================

p = [ 0.04 0 0;
     -0.04 0 0;
     -0.04 0.01 0;
      0.04 0.01 0;
      0.04 0.02 0;
     -0.04 0.02 0;
     -0.04 0.03 0;
      0.04 0.03 0;
      0.04 0.04 0;
     -0.04 0.04 0 ];

R = trotz(pi/2) * trotx(pi);
Td = zeros(4, 4, size(p,1)); 

for i = 1:size(p,1)
    Td(:,:,i) = transl(p(i,:)) * R;
end

mask = [1 1 1 1 1 1];

tol = 1e-6;

lambda = 1;

q_sol = zeros(size(p,1), 6);

% chute inicial
q0 = zeros(1,6);

for i = 1:size(p,1)
    q = KR30.ikine(Td(:,:,i), q0, 'mask', mask, 'tol', tol, 'lambda', lambda);

    q_sol(i,:) = q;

    q0 = q;
end

xyz = zeros(size(q_sol,1), 3);
rpy = zeros(size(q_sol,1), 3);

for i = 1:size(q_sol,1)
    T = KR30.fkine(q_sol(i,:));
    xyz(i,:) = T.t';
    rpy(i,:) = rad2deg(tr2rpy(T));
end

po = zeros(size(p,1), 6);
po(:, 1:3) = (xyz);
po(:, 4:6) = (rpy);
%norm(xyz - p)

rpyd = zeros(size(p,1), 3);
for i = 1:size(p,1)
    rpyd(i,:) = [180 0 90]; 
end
pd = zeros(size(p,1), 6);
po(:, 1:3) = (p - xyz);
po(:, 4:6) = (rpyd - rpy);
%xyz - p



