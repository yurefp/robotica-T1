clear; clc; close all;

%======================================================================
% Configuração do Manipulador
%======================================================================
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
Tab_de = Tat * Ttb_de;

% Transformação Efetuador da Mesa -> Base do Braço
Rab_de = Tab_de.R;
pab_de = Tab_de.t;

Tab_de_inv = [Rab_de.'  -Rab_de.'*pab_de;
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
KR30.base = Tab_de_inv * (transl(0.0, 0.0, 0.0) * trotx(pi));

% Transformação Elo final -> Efetuador
KR30.tool = transl(0, 0, -0.000648) * (trotx(-pi)*trotz(pi/2));

%======================================================================
% Geração da Trajetoria no Espaço Operacional
%======================================================================
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

dxc = 0.02; ddxc = 0.1; tc = 0.2;

pps = 200;
tt = 0;

xd = [0 0 0];
dxd = [0 0 0];
ddxd = [0 0 0];

for i = 1:size(p,1)-1
    % Parâmetros
    q0 = p(i,:);
    qf = p(i+1,:);
    
    dq = norm(qf - q0);
    tf = tc + dq/dxc;
    
    t = linspace(0, tf, tf*pps);
    
    q   = zeros(length(t), length(q0));
    qd  = zeros(length(t), length(q0));
    qdd = zeros(length(t), length(q0));
    
    for k = 1:length(t)
        if t(k) < tc
            % Fase 1: aceleração
            q(k,:)   = q0 + sign(qf - q0) * 0.5*ddxc*t(k)^2;
            qd(k,:)  = sign(qf - q0) * ddxc*t(k);
            qdd(k,:) = sign(qf - q0) * ddxc;
    
        elseif t(k) < tf-tc
            % Fase 2: velocidade constante
            q(k,:)   = q0 + sign(qf - q0) * (0.5*ddxc*tc^2 + dxc*(t(k)-tc));
            qd(k,:)  = sign(qf - q0) * dxc;
            qdd(k,:) = 0;
    
        else
            % Fase 3: desaceleração
            q(k,:)   = qf - sign(qf - q0) * 0.5*ddxc*(tf - t(k))^2;
            qd(k,:)  = sign(qf - q0) * ddxc*(tf - t(k));
            qdd(k,:) = -sign(qf - q0) * ddxc;
        end
    end

    tt = tt + tf;
    xd = [xd; q];
    dxd = [dxd; qd];
    ddxd = [ddxd; qdd];

end

xd = xd(2:end-1, :);
dxd = dxd(2:end-1, :);
ddxd = ddxd(2:end-1, :);

tn = linspace(0, tt, tt*pps);

%======================================================================
% Trajetoria no Espaço das Juntas
%======================================================================
R = trotz(pi/2) * trotx(pi);
Td = zeros(4, 4, length(tn)); 

for i = 1:length(tn)
    Td(:,:,i) = transl(xd(i,:)) * R;
end

mask = [1 1 1 1 1 1];

tol = 1e-6;

lambda = 1;

theta_sol = zeros(length(tn), 6);

% posição inicial
theta0 = [-0.7911 1.1134 -1.7997 1.9726 2.2941 -0.8118];

for i = 1:length(tn)
    theta = KR30.ikine(Td(:,:,i), theta0, 'mask', mask, 'tol', tol, 'lambda', lambda);

    theta_sol(i,:) = theta;

    theta0 = theta;
end

%subplot(1,2,1);
%plot(tn, theta_sol, 'LineWidth', 1.5);
%grid on;
%ylabel('Ângulo (rad)');
%title('Espaço das Juntas');
%num_juntas = size(theta_sol, 2);
%legend(arrayfun(@(x) sprintf('Junta %d', x), 1:num_juntas, 'UniformOutput', false));
%xlabel('Tempo (s)');

%subplot(1,2,2);
plot(tn, xd, 'LineWidth', 1.5);
grid on;
ylabel('Posição (m)');
ylim([-0.05 0.05]);
%title('Espaço Operacional');
legend('x', 'y', 'z');
%num_juntas = size(x, 2);
%legend(arrayfun(@(x) sprintf('Junta %d', x), 1:num_juntas, 'UniformOutput', false));
xlabel('Tempo (s)');

%sgtitle('Trajetórias', 'FontSize', 14, 'FontWeight', 'bold')
