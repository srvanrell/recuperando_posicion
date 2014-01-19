%%
%%
% Obtención de la velocidad y posición por integración de orden cero a
% partir de la aceleración.
%
% Objetivo: comparar con la estimación al utilizar el filtro de kalman.

clc; clear all; close all;

tipo_trayectoria = 'Cuadrado';

% Cargo la aceleración y genero velocidad y posición
a = load(['IMUmovil-' tipo_trayectoria '-data.txt']);

%% Carga de la aceleración de la IMU Quieta
load('IMUQuieta');
a = ACCEL;

%%
bias_acel = 0.0221;
a = a - bias_acel;
a = 9.81 .* a; % 1g --> 9.81 m/s^2
a(:,2) = zeros(size(a,1),1); % anulo gravedad


v = zeros(size(a)); % velocidad
p = zeros(size(a)); % posicion

% Tomo 1 segundo como intervalo de muestreo pero... ¿debería sacarlo de los
% datos del GPS??
dt = 1;
N = size(a,1); % cantidad de muestras


% a = a - repmat(mean(a,1),N,1);
%% Ecuación recursiva de la velocidad
% v(k) = v(k-1) + a(k) * dt

%% Ecuación recursiva de la posición
% p(k) = p(k-1) + v(k) * dt

% TODO Implementación ineficiente (hacerlo con matrices)

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

