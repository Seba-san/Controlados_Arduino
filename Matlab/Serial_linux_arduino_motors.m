%% Configuracion inicial
cd /home/seba/Dropbox/Facultad/Trabajo_final_Controlados/Codigos/Codigos_Arduino/Matlab
cd('C:\Users\Tania\Dropbox\Trabajo final - Controlados\Codigos\Codigos\Matlab')%compu Tania


%% Inicio
% uart = serial('COM2','BaudRate',1200,'DataBits',7);
%s = serial('COM5');
clear all;clc
s=serial('/dev/ttyUSB0','BaudRate',2000000,'DataBits',8);
set(s,'Terminator','CR');
m=10;%5
s.InputBufferSize = 36*m;% antes eran 101 (para el prbs)%Cuantos bytes almaceno durante una operacion de read; Seba: Si no hay dato, el sistema cuelga por timeout.
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
% Seba: Recordar configurar el "m". m determina cuantas muestras se toman
% entre cambios dela PBRS
flushinput(s);%Si hab�a algo en el buffer de entrada,lo limpio.
fprintf(s,1);%Comando para iniciar la prueba
%pause(10);
clear dato tiempo secuencia2;
Encabezado=fscanf(s);
disp(Encabezado);
k=1;
for i=2:s.InputBufferSize
%     disp('lectura')
    salida=str2num(fscanf(s)); % Valor medido
    t=str2num(fscanf(s));%Tiempo en el que se midio
    secuencia=str2num(fscanf(s));%Valor enviado
%     disp('escritura')
   
%     if (length(secuencia)==1 && length(salida)==1 && length(t)==1 )
%         if(salida>=0 && salida<=1023 && t>=0)
            %Solo actualizo los datos si recibo valores validos.
            dato(k)=salida;%*908/1023;%*5/1023;%Acomodo la escala
            tiempo(k)=t;%/1e3;%Acomodo la escala a ms, no us 
            secuencia2(k)=secuencia;
            k=k+1;
%         end
%     end
end

% Grafico
%tiempo=tiempo-tiempo(1);
figure(1);plot(tiempo,dato,'ro')
hold on;plot(tiempo,secuencia2,'g');hold off
title('Respuesta al Escalon del Sistema');
xlabel('tiempo [ms]');ylabel('valor de tension (V)');
save ("medicion_escalon_90_125.mat","tiempo","secuencia2","dato");
% Le digo que espere 5 segundos para volver a hacer la prueba y deje lista
% la medicion. Los 5 seg son para que alcance a descargase el capa
pause(5);
% fprintf(s,1);
disp("listo")
%%  Acondciono los datos

secuencia2=secuencia2-min(secuencia2);
dato=dato-min(dato);


%%
% Modelo caja negra.
% Info sacada de aca: https://la.mathworks.com/help/ident/ug/black-box-modeling.html
% la funcion es tfest: https://la.mathworks.com/help/ident/ref/tfest.html

%load("medicion_escalon_90_125.mat") %load("medicion_escalon.mat")%load('medicion.mat');
np=1; %np es la cantidad de polos, se si no se espesifica los ceros, se supone np-1
iodelay=0; % Es el "tiempo de transporte", copio literal: For discrete-time systems, specify transport delays as integers denoting delay of a multiple of the sample time Ts 
%con el valor "3" y np=3 da una estimacion del 87.2% y con np=4 y el
%delay=4 da 89,12% . Si pones "NaN", le decis que desconoces este parametro
%y el chabon estima solo.
%dato=dato*max(secuencia2)/max(dato);

% calculo la media del tiempo de muestreo
Ts=mean(diff(tiempo))*1e-6; %esta en micros, lo paso a segundos
% transformo la informacion al formato iidata:
data = iddata(dato',secuencia2',Ts);
%data = iddata(dato',secuencia2');
sys = tfest(data,np,[],iodelay,'Ts',data.Ts) 
% sysc = d2c(sys,'zoh');

%%
%C = pidtune(sys,'pid')
PIDF=1; % en 1 es si, en 0 es no
C0 = pid(1,1,1,PIDF,'Ts',data.Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler','TimeUnit','seconds');  
%C = pidtune(sys,C0);
opt = pidtuneOptions('DesignFocus','reference-tracking','CrossoverFrequency',10);%'PhaseMargin',10
[C,info] = pidtune(sys,C0,opt);
%[C,info] = pidtune(sys,C0);
T_pi = feedback(C*sys, 1);
figure (2);
step(T_pi)
title ('Controlado')
figure (3);
step(sys)
title ('Sin controlar')


% ########## PID estandar; lo paso a forma de cociente de polinomios


% ### Lo paso a la forma apropiada para programarlo
mm=C;
Primero=tf(mm.Kp,1);
Segundo=tf([mm.Ts*mm.Ki 0],[1 -1],mm.Ts);
if (PIDF==1)
Tercero=tf(mm.kd*[1 -1],mm.Tf*[1 -1]+mm.Ts*[1 0],mm.Ts);
else
Tercero=tf(mm.kd*[1 -1],mm.Ts*[1 0],mm.Ts); 
end
Pid=Primero+Segundo+Tercero;pipi=Pid; % Supongo que esta en la forma PID paralelo
num=pipi.Numerator{1};
den=pipi.Denominator{1};
num=num/den(1);den=den/den(1); % hay que dividir por den(1) porque es el coeficiente de u(k).
disp('A B C D E')
CUCU=[num -den(2:size(den,2))];
sprintf('%f, %f, %f, %f, %f',CUCU)
%tfsys=tf(sys.Numerator,sys.Denominator,sys.Ts)
% Transformo el sistema discreto a continuo

%% Obtencion de los Coeficientes
% Suponiendo u=a*e(k)+b*e(k-1)+c*e(k-2)+d*u(k-1)+e*u(k-2)

Kp=C.Kp;
Ts=C.Ts;
Kd=C.Kd;Ki=C.Ki;
a=Kp+Ki*Ts+Kd/Ts;
b=-2*Kd/Ts-Kp;
c=Kd/Ts;
d=1;
[a b c d];

% [A,B,C,D]=tf2ss(sys.num,sys.den)
%############Otra Forma
pipi=Pid;
num=pipi.Numerator{1};
den=pipi.Denominator{1};
num=num/den(1);den=den/den(1);
disp('A B C D E')
[num -den(2:size(den,2))]
%% Prueba 2
 
%Para testear el sistema: http://la.mathworks.com/help/control/ref/lsim.html

u = idinput(100) % Generador de PRBS 

% tiempo=tiempo*1e-6;

%%
% Ver respuesta espectral
L=length(dato);
Dens_pot_sal=fftshift(abs(fft(dato))/L);
figure(4)
plot(Dens_pot_sal);title('Espectro de salida')

Dens_pot_in=fftshift(abs(fft(secuencia2))/L);
figure(5)
plot(Dens_pot_in);title('Espectro de entrada')

% Codigo sacado de: https://la.mathworks.com/help/matlab/ref/fft.html?searchHighlight=fft&s_tid=doc_srchtitle
Y = fft(dato);Fs=1/data.Ts;
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')


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

%%
%Pruebas locas
% La idea de esta parte es obtener los coeficientes directmente, sin
% importar la forma.
% ########## Para este caso, la forma es "PIDF"
mm=C_guardado;
Primero=tf(mm.Kp,1);
Segundo=tf([mm.Ts*mm.Ki 0],[1 -1],mm.Ts);
Tercero=tf(mm.kd*[1 -1],mm.Tf*[1 -1]+mm.Ts*[1 0],mm.Ts);
Pidf=Primero+Segundo+Tercero

% ########## PID estandar
mm=C;
Primero=tf(mm.Kp,1);
Segundo=tf([mm.Ts*mm.Ki 0],[1 -1],mm.Ts);
Tercero=tf(mm.kd*[1 -1],mm.Ts*[1 0],mm.Ts);
Pid=Primero+Segundo+Tercero