%%
%%
% Obtenci�n de la velocidad y posici�n por integraci�n de orden cero a
% partir de la aceleraci�n.
%
% Objetivo: comparar con la estimaci�n al utilizar el filtro de kalman.

clc; clear all; close all;

% tipo_trayectoria = 'P1_Encerrado_soloIMU_100Hz';
% tipo_trayectoria = 'P2_Encerrado_GPS_IMU_1Hz';
% tipo_trayectoria = 'P3_Pastoreo_GPS_IMU_1Hz';
% tipo_trayectoria = 'P4_Pastoreo_IMU_100Hz';

% dt = 1;
% xyz2neg(tipo_trayectoria,dt)


load(['a-' tipo_trayectoria '.mat'])
dt = a.dt;
a.g = a.g - 9.81;
a = horzcat(a.norte, a.este, a.g);


v = zeros(size(a)); % velocidad
p = zeros(size(a)); % posicion

N = size(a,1); % cantidad de muestras


% a = a - repmat(mean(a,1),N,1);
%% Ecuaci�n recursiva de la velocidad
% v(k) = v(k-1) + a(k) * dt

%% Ecuaci�n recursiva de la posici�n
% p(k) = p(k-1) + v(k) * dt

% TODO Implementaci�n ineficiente (hacerlo con matrices)

for k = 2:N
    v(k,:) = v(k-1,:) + a(k,:) .* dt;
    p(k,:) = p(k-1,:) + v(k,:) .* dt;
end

%%
subplot(1,2,1)
plot(1:N,p(:,1), 1:N,p(:,2), 1:N,p(:,3))
legend('x','y','z')
subplot(1,2,2)
plot3(p(:,1), p(:,2), p(:,3))

