%%
%%
% Obtenci�n de la velocidad y posici�n por integraci�n de primer orden a
% partir de la aceleraci�n.
%
% Objetivo: comparar con la estimaci�n al utilizar el filtro de kalman.

clc; clear all; close all;

tipo_trayectoria = 'P3_Pastoreo_GPS_IMU_1Hz';

% Cargo la aceleraci�n y genero velocidad y posici�n
acc = load(['IMUaceleracion-' tipo_trayectoria '.txt']);
dir = load(['IMUdirecciones-' tipo_trayectoria '.txt']);

nor = dir(:,1:3);
gra = dir(:,4:6);

N = size(acc,1);

% Aceleracion
a.x = acc(:,1);
a.y = acc(:,2);
a.z = acc(:,3);

% Norte
norte.x = dir(:,1);
norte.y = dir(:,2);
norte.z = dir(:,3);

% Gravedad
g.x = dir(:,4);
g.y = dir(:,5);
g.z = dir(:,6);

%%
norte.x = norte.x - norte.x .* g.x;
norte.y = norte.y - norte.y .* g.y;
norte.z = norte.z - norte.z .* g.z;

%% Corroborando normalidad de los vectores
% ones(size(g.x)) - sqrt(g.x .^2 + g.y .^2 + g.z .^2)
% 
% ones(size(norte.x)) - sqrt(norte.x .^2 + norte.y .^2 + norte.z .^2)

%% Corroborando ortogonalidad entre la gravedad y el norte
% dot(horzcat(norte.x,norte.y,norte.z), horzcat(g.x,g.y,g.z),2)

%% Obtengo el este por producto cruz
est = cross(horzcat(g.x,g.y,g.z), horzcat(norte.x,norte.y,norte.z));
este.x = est(:,1); 
este.y = est(:,2);
este.z = est(:,3);

%% Corroborando normalidad del este
% ones(size(este.x)) - sqrt(este.x .^2 + este.y .^2 + este.z .^2)


%% Proyecto la aceleracion sobre los ejes norte, este y gravedad
a.norte = dot(acc,nor,2);
a.este  = dot(acc,est,2);
a.grav  = dot(acc,gra,2);

%% Variación de los ejes en el tiempo
% close all
% hold on
% cero=zeros(size(g.x));
% iter=(1:N)';
% n = 1;
% quiver3(iter(1:n), cero(1:n), cero(1:n), g.x(1:n), g.y(1:n), g.z(1:n),0.5,'blue')
% quiver3(iter(1:n), cero(1:n), cero(1:n), norte.x(1:n), norte.y(1:n), norte.z(1:n),0.5,'red')
% quiver3(iter(1:n), cero(1:n), cero(1:n), este.x(1:n), este.y(1:n), este.z(1:n),0.5,'black')
% xlabel('x')
% ylabel('y')
% zlabel('z')
% legend('g','norte','este')


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


%%
a = horzcat(a.norte, a.este, a.grav);
%%
% bias_acel = 0.0223;
% a = a - bias_acel;
a = 9.81 .* a; % 1g --> 9.81 m/s^2
% a(:,2) = zeros(size(a,1),1); % anulo gravedad
%%
v = zeros(size(a)); % velocidad
p = zeros(size(a)); % posicion
%%
% Tomo 1 segundo como intervalo de muestreo pero... �deber�a sacarlo de los
% datos del GPS??
dt = 1;
N = size(a,1); % cantidad de muestras
%% Ecuaci�n recursiva de la velocidad
% v(k) = v(k-1) + a(k-1) * dt + 0.5 * (a(k) - a(k-1)) * dt
% v(k) = v(k-1) + 0.5 * (a(k) + a(k-1)) * dt

%% Ecuaci�n recursiva de la posici�n
% p(k) = p(k-1) + v(k) * dt + 0.5 * (v(k) - v(k-1)) * dt
% p(k) = p(k-1) + 0.5 * (v(k) + v(k-1)) * dt
% p(k) = p(k-1) + 0.5 * ((0.5 * (a(k) + a(k-1)) * dt) + (0.5 * (a(k-1) + a(k-2)) * dt)) * dt
% p(k) = p(k-1) + (0.25 * a(k) + 0.5 * a(k-1) + 0.25 * a(k-2)) * dt^2

% TODO Implementaci�n ineficiente (hacerlo con matrices)

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
subplot(1,2,1)
plot(1:N,p(:,1), 1:N,p(:,2), 1:N,p(:,3))
legend('x','y','z')
subplot(1,2,2)
plot3(p(:,1), p(:,2), p(:,3))

