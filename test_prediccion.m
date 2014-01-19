clear all; close all; clc;
%% TODO
% - No funciona. Al menos no se ve el cuadrado esperado
% - como calculo la varianza del error de medicion??
% - �Qu� hacer con la gravedad? hay que filtrarla? o agregarla al modelo?
% - �Mejora al tomar tiempos del GPS en vez de 1 segundo?
% - Revisar definici�n de R (covarianza del ruido de medici�n)

%% Cargo los datos de aceleraci�n

tipo_trayectoria = 'Cuadrado';

a_med = load(['IMUmovil-' tipo_trayectoria '-data.txt']); % aceleracion medida
% a_med = ACCEL;
% bias_acel = 0.0221;
% a_med = a_med - bias_acel;
a_med = 9.81 .* a_med; % 1g --> 9.81 m/s^2
% a_med(:,2) = zeros(size(a_med,1),1); % para eliminar la componente de la
% gravedad?



N=size(a_med,1);






t=1:N; % vector de tiempos dale que se tomaron muestras cada 1 segundo



%% Sistema
% x(k+1) = A x(k) + B u(k) + w(k)
% y(k+1) = H x(k) + v(k)
%
% Suposiciones
% -------------
% B = 0
% 
% Q = E(w*w')
% R = E(v*v')
% E(w)=E(v)=0
% A veces H la llaman C (por ejemplo en la ayuda de matlab)

%% Varianza en el error de medici�n
% Estimada a partir del registro de la IMUquieta
load('IMUQuieta');
% mean(ACCEL)
bias_acel = 0.0223;
% var_error_med = var((ACCEL-bias_acel) * 9.81);  % 1g --> 9.81 m/s^2

a_med2 = a_med - repmat(mean(a_med,1),N,1);
var_error_med = var(a_med2(1:20,:));


%% estados
% [p_x, v_x, a_x, p_y, v_y, a_y, p_z, v_z, a_z]

x0 = zeros(9,1);        % Estado inicial nulo
dt = 1;                 % Intervalo de muestreo (1 s)
                        
%% Reservo espacio para los estados estimados

% Estimaciones con el kalman de matlab
x1 = zeros(length(x0),N); % ver x0

% Estimaciones con kalman propio (time varying kalman filter)
x = zeros(length(x0),N); % ver x0
a_est = zeros(size(a_med));


%% Matriz de transici�n de estados (A)
% Se considera aceleraci�n constante en el intervalo
% Movimiento rectil�neo uniformemente acelerado
% p(k) = p(k-1) + v(k) * dt + 0.5 * a(k) * dt^2
% v(k) = v(k-1) + a(k) * dt
% a(k) = a(k-1) 

Ai = [ 1  dt 0.5*dt.^2 ; 
       0   1    dt     ; % Matriz de transici�n para un solo eje
       0   0    1     ];

A = blkdiag(Ai,Ai,Ai);   % Matriz de transici�n

%% Matriz de observaci�n (H, la ayuda de Matlab le llama C)
% Relaciona los estados con las mediciones
% Me quedo solo con la aceleracion

Hi = [0  0  1]; % Matriz de observaci�n para un solo eje

H = blkdiag(Hi,Hi,Hi);  % Matriz de observaci�n

%% Matriz de covarianza del ruido del proceso ( Q = E(w*w') )
% A causa de la integraci�n, el ruido presente en las mediciones de la
% aceleraci�n se propagan a la velocidad y la posici�n.
% El filtro de kalman utiliza esta matriz para reducir el efecto.

% elijo la matriz inicial como en filieri melchiotti, salvo que tomo cada
% eje de forma independiente, con un qc distinto
% TODO revisar la referencia 20 para corroborar

%The definition is reduced to a single parameter qc which is scalar and 
% assumed equal to the square of the error variance.

qc_x = var_error_med(1); % Varianza del error de medicion en el eje x
qc_y = var_error_med(2); % Varianza del error de medicion en el eje y
qc_z = var_error_med(3); % Varianza del error de medicion en el eje z

Qi = [ 1/20*dt^5  1/8*dt^4  1/6*dt^3  ; 
        1/8*dt^4  1/3*dt^3  1/2*dt^2  ; % Matriz (o casi) para un solo eje
        1/6*dt^3  1/2*dt^2    1*dt   ];
Qx = qc_x^2 * Qi;
Qy = qc_y^2 * Qi;
Qz = qc_z^2 * Qi;
    
Q = blkdiag(Qx,Qy,Qz);  % Matriz de covarianza del ruido del proceso


%% Matriz de covarianza del ruido de medici�n ( R = E(v*v') )
% Los supongo independientes??

R = diag(var_error_med); % Varianza del error de medicion en cada eje


%% Matriz de covarianza del error de estimaci�n de los estados (P)
% Como se actualizar� en cada iteraci�n la suposici�n inicial es que sea la
% matriz identidad. El equivocarnos dilata la convergencia pero a largo
% plazo deber�a funcionar.

P = eye(9);





%% time variant kalman filter
for j=2:N
    % Predicci�n
    x(:,j) = A * x(:,j-1); % x[n|n-1]
    P = A * P * A' + Q;    % P[n|n-1]

    % Ganancia del filtro de Kalman:
    K = P * H' /(H * P * H' + R); % / hace la inversa
    
    % Correcci�n basado en la observaci�n (a_med)
    x(:,j) = x(:,j) + K * (a_med(j)' - H * x(:,j));   % x[n|n] 
    P = P - K * H * P; % ==(eye(9)-K*H)*P             % P[n|n]
    
    a_est(j,:) = ( H * x(:,j) )'; % salida estimada del sistema
    
% figure(2)
%     hold on
%     plot(j,K(1),j,K(2),j,K(3),j,K(4),j,K(5),j,K(6))
end

%%
figure(1)
eje = ['x'; 'y'; 'z'];
for j = 1:3:7
    subplot(3,3,j), plot(t,x(j,:),'--'); axis tight
    title(['Posicion en el eje ' eje((j+2)/3)])
    xlabel('No. de muestras'), ylabel('Posicion')

    subplot(3,3,j+1), plot(t,x(j+1,:),'--'); axis tight
    title(['Velocidad en el eje ' eje((j+2)/3)])
    xlabel('No. de muestras'), ylabel('Velocidad')

    subplot(3,3,j+2), plot(t,a_med(:,(j+2)/3),'--',t,x(j+2,:),'-'); axis tight
    title(['Aceleracion en el eje ' eje((j+2)/3)])
    xlabel('No. de muestras'), ylabel('Aceleracion')
end