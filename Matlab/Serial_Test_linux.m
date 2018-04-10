%% Inicio
% uart = serial('COM2','BaudRate',1200,'DataBits',7);
%s = serial('COM5');
clear all;clc
s=serial('/dev/ttyUSB0','BaudRate',2000000,'DataBits',8);
set(s,'Terminator','CR');
s.InputBufferSize = 101;%Cuantos bytes almaceno durante una operacion de read; Seba: Si no hay dato, el sistema cuelga por timeout.
fopen(s)
disp('Puerto Abierto')
%% Fin
fclose(s)
%clear all;clc
disp('Puerto Cerrado')
%% Tx y Rx
%Tania: no me estaba respondiendo a cambios del sistema. Por eso prob� con
%limpiar el buffer antes de transmitir y recibir, pero despu�s se arregl� y
%se mantuvo as� aunque le coment� el comando. Por ah� era cuesti�n de la
%placa, pero si vuelve el problema acordate de esto.
flushinput(s);%Si hab�a algo en el buffer de entrada,lo limpio.
fprintf(s,1);%Comando para iniciar la prueba
%pause(10);
clear dato tiempo secuencia2;
Encabezado=fscanf(s);
disp(Encabezado);
k=1;
for i=2:s.InputBufferSize
    salida=str2num(fscanf(s)); % Valor medido
    t=str2num(fscanf(s));%Tiempo en el que se midio
    secuencia=str2num(fscanf(s))*908/1023;%Valor enviado
   
   
%     if (length(secuencia)==1 && length(salida)==1 && length(t)==1 )
%         if(salida>=0 && salida<=1023 && t>=0)
            %Solo actualizo los datos si recibo valores validos.
            dato(k)=salida;%*5/1023;%Acomodo la escala
            tiempo(k)=t;%/1e3;%Acomodo la escala a ms, no us 
            secuencia2(k)=secuencia;
            k=k+1;
%         end
%     end
end

% Grafico
figure(1);plot(tiempo,dato,'ro')
hold on;plot(tiempo,secuencia2,'g');hold off
title('Respuesta al Escalon del Sistema');
xlabel('tiempo [ms]');ylabel('valor de tension (V)');
save ("medicion.mat","tiempo","secuencia2","dato");
% Le digo que espere 5 segundos para volver a hacer la prueba y deje lista
% la medicion. Los 5 seg son para que alcance a descargase el capa
pause(5);
fprintf(s,1);
disp("listo")
%% 
% Modelo caja negra.
% Info sacada de aca: https://la.mathworks.com/help/ident/ug/black-box-modeling.html
% la funcion es tfest: https://la.mathworks.com/help/ident/ref/tfest.html

load ("medicion.mat");
np=3; %np es la cantidad de polos, se si no se espesifica los ceros, se supone np-1
iodelay=NaN; % Es el "tiempo de transporte", copio literal: For discrete-time systems, specify transport delays as integers denoting delay of a multiple of the sample time Ts 
%con el valor "3" y np=3 da una estimacion del 87.2% y con np=4 y el
%delay=4 da 89,12% . Si pones "NaN", le decis que desconoces este parametro
%y el chabon estima solo.
% calculo la media del tiempo de muestreo
Ts=mean(diff(tiempo))*1e-6; %esta en micros, lo paso a segundos
% transformo la informacion al formato iidata:
data = iddata(dato',secuencia2',Ts);
sys = tfest(data,np,[],iodelay,'Ts',data.Ts) %devuelve el sistema en tiempo continuo
C = pidtune(sys,'pid')
% NO SE PORQUE NO FUNCIONAAAAAAAAAAAAAAAAAAAAAAAAAA da tiempos muy
% prolongados. 
T_pi = feedback(C*sys, 1);
step(T_pi)



C0 = pid(1,1,1,'Ts',data.Ts,'IFormula','BackwardEuler','TimeUnit','seconds');  
C = pidtune(sys,C0)


%tfsys=tf(sys.Numerator,sys.Denominator,sys.Ts)
% Transformo el sistema discreto a continuo
%sysc = d2c(sys,'zoh');



%% Intento de tangente
%Variable indep:
t=cell2mat(tiempo(6:length(tiempo)));
%Variable dep:
y=cell2mat(dato(6:length(tiempo)));

%C�digo sacado de internet:
d1y = gradient(y,t);	% Numerical Derivative
d2y = del2(y,t);	% Numerical Second Derivative

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Como me tiraba error le pongo este parche dudoso que encontr� por ah�
%[x, index] = unique(x); 
%yi = interp1(x, y(index), xi); 
[d1y2, index] = unique(d1y); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%t_infl = interp1(d1y, t, max(d1y));	% Find �t� At Maximum Of First Derivative
t_infl = interp1(d1y2, t(index), max(d1y));%Modificado
y_infl = interp1(t, y, t_infl);     % Find �y� At Maximum Of First Derivative
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
%el punto donde la recta alcanza el m�ximo de la respuesta al escal�n (ti2)
%y el punto donde alcanza el m�nimo (ti1), que ser�a el equivalente a
%cortar al eje x si no hubiese offset.
ti1=(min(y)-intcpt)/slope;
ti2=(max(y)-intcpt)/slope;
L=ti1-t(1);
T=ti2-ti1;
K=max(y)-min(y);
%% C�lculo del PID
Kp=1.2*T/(L*K);
Ti=2*L;
Td=0.5*L;
%%
%Pruebas locas

for i=1:351
str2num(fscanf(s))

end

