clear; clc; close all;

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

subplot(3,1,1);
plot(tn,xd(:,2), 'LineWidth', 1.5);
grid on;
ylabel('Posição (m)');
ylim([-0.01 0.05]);

subplot(3,1,2);
plot(tn,dxd(:,2), 'LineWidth', 1.5);
grid on;
ylabel('Velocidade (m/s)');
ylim([-0.005 0.025]);

subplot(3,1,3);
plot(tn,ddxd(:,2), 'LineWidth', 1.5);
grid on;
ylabel('Aceleração (m/s^2)');
ylim([-0.12 0.12]);

xlabel('Tempo (s)');

%subplot(3,2,2), plot(tn,xd(:,2)),  grid on 
%subplot(3,2,4), plot(tn,dxd(:,2)), grid on
%subplot(3,2,6), plot(tn,ddxd(:,2)),grid on
%xlabel('Em y')

