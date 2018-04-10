%% Prueba de linealidad
%% Configuracion inicial
cd ~/Dropbox/Facultad/Trabajo_final_Controlados/Codigos/Codigos/Matlab



%% Inicio
% uart = serial('COM2','BaudRate',1200,'DataBits',7);
%s = serial('COM5');
clear all;clc
s=serial('/dev/ttyUSB0','BaudRate',2000000,'DataBits',8);
set(s,'Terminator','CR');

s.InputBufferSize = 245;%Cuantos bytes almaceno durante una operacion de read; Seba: Si no hay dato, el sistema cuelga por timeout.
fopen(s)
disp('Puerto Abierto')
%% Fin
fclose(s)
%clear all;clc
disp('Puerto Cerrado')
%%

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
    
    secuencia=str2num(fscanf(s))%Valor enviado
%     disp('escritura')
   
%     if (length(secuencia)==1 && length(salida)==1 && length(t)==1 )
%         if(salida>=0 && salida<=1023 && t>=0)
            %Solo actualizo los datos si recibo valores validos.
            dato(k)=salida;%*908/1023;%*5/1023;%Acomodo la escala
           
            secuencia2(k)=secuencia;           
            k=k+1;
%         end
%     end
end
hold off
% Grafico
 figure(1);title ('respuesta')
 plot(secuencia2,dato,'ro')
%figure(1);plot(tiempo,dato,'ro')
%hold on;plot(tiempo,secuencia2,'g');hold off
%title('Respuesta al Escalon del Sistema');
%xlabel('tiempo [ms]');ylabel('valor de tension (V)');
save ("medicion_linealidad_linealizado2.mat","secuencia2","dato");
% Le digo que espere 5 segundos para volver a hacer la prueba y deje lista
% la medicion. Los 5 seg son para que alcance a descargase el capa
%pause(5);
% fprintf(s,1);
disp("listo")
%% 
%Obtengo la funcion 
load("medicion_linealidad.mat")
plot(secuencia2',dato','rx')
% Saco los puntos que estan por debajo de un umbral
%%
umbral=25;
exclude1= dato<umbral;
diodoeqn='a*(exp(b*x))+c';


%f=fit(dato',secuencia2','poly4','exclude',exclude1)
f=fit(secuencia2',dato',diodoeqn)%,'exclude',exclude1)
%plot(f,dato',secuencia2',exclude1)
plot(f,secuencia2',dato')
ylim([0 1024])
%%
mi=min(dato);mx=max(dato);
paso=(mx-mi)/255;
x=0:1:255;
xp=x.*paso;
res=zeros(1,256);
ex=zeros(1,256);
for i=1:255
busqueda1=xp(i)-dato;
m=find(min(abs(busqueda1))==abs(busqueda1),1,'last');
res(i)=dato(m);
ex(i)=m;
end
plot(xp,res,'bx')


%%
% escribo en archivo para pegarlo en arduino
fileID = fopen('lista.txt','w');
formato="%u,";
fprintf(fileID,formato,res)
fclose(fileID);



%%
iguales=diff(res);
n=1;
for i=1:255
if(iguales(i)~=0)
    filtrado(n)=res(i);
    n=n+1;
end
end

xpp=0:1:size(filtrado,2)-1;
plot(xpp,filtrado,'bx')
%%
% escribo en archivo para pegarlo en arduino
fileID = fopen('lista_filtrado.txt','w');
formato="%u,";
fprintf(fileID,formato,ex)
fclose(fileID);

