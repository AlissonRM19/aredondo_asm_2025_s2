%% ================================================
% EXTRACCIÓN DE MODELO FÍSICO ORDEN 2
% ================================================
clc; clear; close all;

%% 1. Cargar datos
data = readmatrix("datos.txt");

t = data(:,1);
y = data(:,2);

%% 2. Graficar respuesta real
figure('Name','Respuesta real','Color','white');
plot(t, y, 'b', 'LineWidth', 2); grid on;
xlabel('Tiempo (s)');
ylabel('Salida (cm)');
title('Respuesta al Escalón Real');

%% 3. Ganancia estática
K = mean(y(end-5:end));
fprintf("Ganancia real K = %.4f\n", K);

%% 4. Normalizar
y_norm = y / K;

%% 5. Overshoot (valor máximo)
[val_max, idx_max] = max(y_norm);
t_peak = t(idx_max);

Mp = val_max - 1;

fprintf("Overshoot Mp = %.4f (%.2f %%)\n", Mp, Mp*100);
fprintf("Tiempo al pico tp = %.4f s\n", t_peak);

if Mp <= 0
    fprintf("\nLa respuesta NO tiene overshoot.\n");
    fprintf("El sistema NO es de 2do orden subamortiguado.\n");
    fprintf("Modelo correcto: **primer orden**.\n");
    return;
end

%% 6. Encontrar ζ numéricamente
% Probamos valores de zeta entre 0 y 1
zeta_values = linspace(0.01,0.99,5000);
Mp_teorico = exp(-zeta_values*pi ./ sqrt(1 - zeta_values.^2));

% Elegimos el zeta cuyo Mp teórico más se parece al Mp real
[~, idx] = min(abs(Mp_teorico - Mp));
zeta = zeta_values(idx);

fprintf("Zeta ≈ %.4f\n", zeta);

%% 7. Calcular frecuencia natural ωn
omega_n = pi / (t_peak * sqrt(1 - zeta^2));
fprintf("ω_n ≈ %.4f rad/s\n", omega_n);

%% 8. Función de transferencia (segundo orden)
num = K * omega_n^2;
den = [1  2*zeta*omega_n  omega_n^2];

fprintf("\nModelo de segundo orden:\n");
fprintf("      %.4f\n", num);
fprintf("G(s)= -------------------------------\n");
fprintf("     s^2 + %.4f s + %.4f\n", 2*zeta*omega_n, omega_n^2);

%% 9. Modelo estimado vs real
wn = omega_n;
z = zeta;

y_model = K * (1 - (1/sqrt(1-z^2)) * exp(-z*wn*t) .* ...
               sin(wn*sqrt(1-z^2)*t + acos(z)));

figure('Name','Comparación real vs modelo','Color','white');
plot(t, y,'b','LineWidth',2); hold on;
plot(t, y_model,'r--','LineWidth',2);
grid on;
xlabel('Tiempo (s)');
ylabel('Salida (cm)');
legend('Datos reales','Modelo estimado');
title('Comparación entre datos y modelo de segundo orden');
