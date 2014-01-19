%%
% Obtencion de la velocidad y posicion por integracion de primer orden a
% partir de la aceleracion.
%
% Objetivo: comparar con la estimacion al utilizar el filtro de kalman.

clc; clear all; close all;

% tipo_trayectoria = 'P1_Encerrado_soloIMU_100Hz';
% tipo_trayectoria = 'P2_Encerrado_GPS_IMU_1Hz';
tipo_trayectoria = 'P3_Pastoreo_GPS_IMU_1Hz';
% tipo_trayectoria = 'P4_Pastoreo_IMU_100Hz';

% dt = 1;
% xyz2neg(tipo_trayectoria,dt)


load(['a-' tipo_trayectoria '.mat'])
dt = a.dt;
a.g = a.g - 9.81;
a = horzcat(a.norte, a.este, a.g);



%% Vectores a llenar con la velocidad y posicion
v = zeros(size(a)); % velocidad
p = zeros(size(a)); % posicion

%%
N = size(a,1); % cantidad de muestras
%% Ecuacion recursiva de la velocidad
% v(k) = v(k-1) + a(k-1) * dt + 0.5 * (a(k) - a(k-1)) * dt
% v(k) = v(k-1) + 0.5 * (a(k) + a(k-1)) * dt

%% Ecuacion recursiva de la posicion
% p(k) = p(k-1) + v(k) * dt + 0.5 * (v(k) - v(k-1)) * dt
% p(k) = p(k-1) + 0.5 * (v(k) + v(k-1)) * dt
% p(k) = p(k-1) + 0.5 * ((0.5 * (a(k) + a(k-1)) * dt) + (0.5 * (a(k-1) + a(k-2)) * dt)) * dt
% p(k) = p(k-1) + (0.25 * a(k) + 0.5 * a(k-1) + 0.25 * a(k-2)) * dt^2

% TODO Implementacion ineficiente (hacerlo con matrices)

for k = 1:N
    if k<2
        v(k,:) = 0.5 * (a(k,:)) .* dt;
        p(k,:) = (0.25 * a(k,:)) .* dt^2;
    elseif k<3
        v(k,:) = 0.5 * (a(k,:) + a(k-1,:)) .* dt;
        p(k,:) = p(k-1,:) + (0.25 * a(k,:) + 0.5 * a(k-1,:)) .* dt^2;
    else
        v(k,:) = 0.5 * (a(k,:) + a(k-1,:)) .* dt;
        p(k,:) = p(k-1,:) + (0.25 * a(k,:) + 0.5 * a(k-1,:) + 0.25 * a(k-2,:)) * dt^2;
    end
end

%%
figure(1)
subplot(1,2,1)
plot(1:N,a(:,1), 1:N,a(:,2), 1:N,a(:,3))
title('Aceleracion')
legend('norte','este','gravedad')
subplot(1,2,2)
plot3(a(:,1), a(:,2), a(:,3))
xlabel('norte')
ylabel('este')
zlabel('gravedad')

figure(2)
subplot(1,2,1)
plot(1:N,v(:,1), 1:N,v(:,2), 1:N,v(:,3))
title('Velocidad')
legend('norte','este','gravedad')
subplot(1,2,2)
plot3(v(:,1), v(:,2), v(:,3))
xlabel('norte')
ylabel('este')
zlabel('gravedad')

figure(3)
subplot(1,2,1)
plot(1:N,p(:,1), 1:N,p(:,2), 1:N,p(:,3))
title('Posicion')
legend('norte','este','gravedad')
subplot(1,2,2)
plot3(p(:,1), p(:,2), p(:,3))
xlabel('norte')
ylabel('este')
zlabel('gravedad')


%% Variacion de los ejes en el tiempo
% close all
% hold on
% cero=zeros(size(g.x));
% iter=(1:N)';
% n = 100;
% quiver3(iter(1:n)', cero(1:n)', cero(1:n)', g.x(1:n)', g.y(1:n)', g.z(1:n)',0,'blue')
% quiver3(iter(1:n), cero(1:n), cero(1:n), norte.x(1:n), norte.y(1:n), norte.z(1:n),0,'red')
% quiver3(iter(1:n), cero(1:n), cero(1:n), este.x(1:n), este.y(1:n), este.z(1:n),0,'black')
% xlabel('x')
% ylabel('y')
% zlabel('z')
% legend('g','norte','este')

%%
% close all; 
% hold on; 
% plot(g.x,'b'); 
% plot(g.y,'r'); 
% plot(g.z,'g'); 
% plot(sqrt(g.x.^2+g.y.^2+g.z.^2),'k')


%% Variacion de la aceleracion en el tiempo respecto a los ejes norte, este, g
% close all
% hold on
% cero=zeros(size(g.x));
% iter=(1:N)';
% n = 100;
% quiver3(iter(1:n), cero(1:n), cero(1:n), a.norte(1:n), a.este(1:n), a.grav(1:n),0.5,'blue')
% xlabel('norte')
% ylabel('este')
% zlabel('grav')
% legend('aceleracion')

