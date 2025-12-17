clc; clearvars; close all;

s = tf('s');

funcaoTensao = 180.0/(2.82*10^(-7)*s^2+0.00047*s+6.0);
realimentacao = 3.9/(3.9+15);

funcaoReal = funcaoTensao*realimentacao;

controlador = 124.1/s;

fechado = feedback(funcaoTensao*controlador,realimentacao);
step(fechado)

controladorD = c2d(controlador,1/50E3,'tustin');
[num, den] = tfdata(controladorD, 'v');
