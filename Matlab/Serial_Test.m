%% Inicio
% uart = serial('COM2','BaudRate',1200,'DataBits',7);
%s = serial('COM5');
clear all;clc
s=serial('COM6','BaudRate',115200,'DataBits',8);
set(s,'Terminator','CR');
s.InputBufferSize = 70;%Cuántos bytes almaceno durante una operación de read
fopen(s)
%% Tx y Rx
%Tania: no me estaba respondiendo a cambios del sistema. Por eso probé con
%limpiar el buffer antes de transmitir y recibir, pero después se arregló y
%se mantuvo así aunque le comenté el comando. Por ahí era cuestión de la
%placa, pero si vuelve el problema acordate de esto.
flushinput(s);%Si había algo en el buffer de entrada,lo limpio.
fprintf(s,1);%Comando para iniciar la prueba
clear dato tiempo muestra t;
Encabezado=fscanf(s);
disp(Encabezado);
k=1;
for i=2:s.InputBufferSize
    muestra=str2num(fscanf(s));%Valor medido
    t=str2num(fscanf(s));%Tiempo en el que se midió
    if (length(muestra)==1 && length(t)==1)
        if(muestra>=0 && muestra<=1023 && t>=0)
            %Sólo actualizo los datos si recibí valores válidos.
            dato{k}=muestra*5/1023;%Acomodo la escala
            tiempo{k}=t/1e3;%Acomodo la escala a ms, no us    
            k=k+1;
        end
    end
end
% Gráfico
figure(1);plot(cell2mat(tiempo),cell2mat(dato),'ro')
hold on;plot(cell2mat(tiempo),cell2mat(dato),'g');hold off
title('Respuesta al Escalón del Sistema');
xlabel('tiempo [ms]');ylabel('valor de tensión (V)');
%% Intento de tangente
%Variable indep:
t=cell2mat(tiempo(6:length(tiempo)));
%Variable dep:
y=cell2mat(dato(6:length(tiempo)));

%Código sacado de internet:
d1y = gradient(y,t);	% Numerical Derivative
d2y = del2(y,t);	% Numerical Second Derivative

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Como me tiraba error le pongo este parche dudoso que encontré por ahí
%[x, index] = unique(x); 
%yi = interp1(x, y(index), xi); 
[d1y2, index] = unique(d1y); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%t_infl = interp1(d1y, t, max(d1y));	% Find ‘t’ At Maximum Of First Derivative
t_infl = interp1(d1y2, t(index), max(d1y));%Modificado
y_infl = interp1(t, y, t_infl);     % Find ‘y’ At Maximum Of First Derivative
%slope  = interp1(t, d1y, t_infl);   % Slope Defined Here As Maximum Of First Derivative
[t2, index] = unique(t); %Modificado
slope  = interp1(t2, d1y(index), t_infl);%Modificado

slope  = interp1(t, d1y, t_infl);
intcpt = y_infl - slope*t_infl;     % Calculate Intercept
tngt = slope*t + intcpt;            % Calculate Tangent Line

figure(1)
plot(t, y)
hold on
plot(t, d1y, '-.m',    t, d2y, '--c')% Plot Derivatives (Optional)
plot(t, tngt, '-r', 'LineWidth',1)   % Plot Tangent Line
plot(t_infl, y_infl, 'bp')           % Plot Maximum Slope

plot(cell2mat(tiempo),cell2mat(dato),'ro')%Modificado

hold off
grid
legend('y(t)', 'dy/dt', 'd^2y/dt^2', 'Tangent', 'Location','E')
axis([xlim    min(min(y),intcpt)  ceil(max(y))])

%Mio:
%Sabiendo que la ec de la recta tangente es: tngt = slope*t + intcpt, busco
%el punto donde la recta alcanza el máximo de la respuesta al escalón (ti2)
%y el punto donde alcanza el mínimo (ti1), que sería el equivalente a
%cortar al eje x si no hubiese offset.
ti1=(min(y)-intcpt)/slope;
ti2=(max(y)-intcpt)/slope;
L=ti1-t(1);
T=ti2-ti1;
K=max(y)-min(y);
%% Cálculo del PID
Kp=1.2*T/(L*K);
Ti=2*L;
Td=0.5*L;
%% Fin
fclose(s)
%clear all;clc
disp('Puerto Cerrado')