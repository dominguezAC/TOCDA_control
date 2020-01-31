
num = [0 0 0 108 48667]
den = [0 480 14840 181560 22580]
%cálculo de polos y ceros
zeros = roots (num)
polos = roots (den)
%polos = pole(funciontrans) %mismo cálculo que la fila anterior

funciontrans = tf(num,den) %función de transferencia
zpk(funciontrans) %continuouns-time transfer function
damp(funciontrans) %continuous-time zero/pole/gain model


subplot(2,2,1),pzmap(num,den) % dibuja los polos y ceros
subplot(2,2,2),step(funciontrans) % dibuja la respueta al escalon 


%DIAGRAMA DE BODE y NIQUIST
subplot(2,2,3),bode(funciontrans)
subplot(2,2,4),nyquist(funciontrans)