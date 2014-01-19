function xyz2neg(tipo_trayectoria,dt)
%xyz2neg Cambia las coordenadas de referencia de la aceleracion
% Las aceleraciones dadas en las coordenadas relativas a la IMU (x,y,z)
% pasan a estar referenciadas respecto al norte, este y la gravedad (n,e,g)
%
% Los archivos necesarios son IMUaceleracion-<tipo_trayectoria>.txt e
% IMUdirecciones-<tipo_trayectoria>.txt
%%
% tipo_trayectoria = 'P4_Pastoreo_IMU_100Hz';
a.dt = dt;

% Cargo la aceleracion y las direcciones de referencia
axyz = load(['IMUaceleracion-' tipo_trayectoria '.txt']); % aceleracion
dirxyz = load(['IMUdirecciones-' tipo_trayectoria '.txt']);
norxyz = dirxyz(:,1:3); % norte
gxyz = dirxyz(:,4:6); % gravedad

N = size(axyz,1);

% Aceleracion en (x,y,z)
a.x = axyz(:,1);
a.y = axyz(:,2);
a.z = axyz(:,3);

% Norte en (x,y,z)
norte.x = dirxyz(:,1);
norte.y = dirxyz(:,2);
norte.z = dirxyz(:,3);

% Gravedad en (x,y,z)
g.x = dirxyz(:,4);
g.y = dirxyz(:,5);
g.z = dirxyz(:,6);

%% Obtengo el este en (x,y,z) por producto cruz
estxyz = cross(gxyz, norxyz);
este.x = estxyz(:,1); 
este.y = estxyz(:,2);
este.z = estxyz(:,3);

%% Corroborando normalidad de los vectores
% plot(ones(size(g.x)) - sqrt(g.x .^2 + g.y .^2 + g.z .^2))
% plot(ones(size(norte.x)) - sqrt(norte.x .^2 + norte.y .^2 + norte.z .^2))
% plot(ones(size(este.x)) - sqrt(este.x .^2 + este.y .^2 + este.z .^2))

%% Corroborando ortogonalidad entre la gravedad y el norte
% plot(dot(norxyz, gxyz,2))
% plot(dot(norxyz, estxyz,2))
% plot(dot(estxyz, gxyz,2))

%% Proyecto la aceleracion sobre los ejes norte, este y gravedad
a.norte = dot(axyz,norxyz,2);
a.este  = dot(axyz,estxyz,2);
a.g     = dot(axyz,gxyz,2);

%% Guardo la aceleracion en m/s^2
gravedad = 9.81; % 1g --> 9.81 m/s^2

a.norte = a.norte * gravedad;
a.este = a.este * gravedad;
a.g = a.g * gravedad;
a.x = a.x * gravedad;
a.y = a.y * gravedad;
a.z = a.z * gravedad;

save(['a-' tipo_trayectoria '.mat'],'a')

end




