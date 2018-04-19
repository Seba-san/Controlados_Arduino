function [Dato]=DatoRx(s)
% Defino la trama como:

% [inicio][cantidad de datos][identificador][Datos][fin]
% inicio= 0xff;
%identificador= a definir
%fin= 0xfe
%un identificador del dato y el dato en si,
% finalizado con un retorno de lina CR.
tic;a=1;
while(s.BytesAvailable<2)
    t=toc;
    if t>a
        tic;
        disp ('Esperando a que llegue algo')
    end
end
tic;a=1;

while(255~=str2double(fscanf(s)))
    t=toc;
    if t>a
        tic;
        disp ('Esperando a que llegue el inicio')
    end
end

cantidad=str2double(fscanf(s));
identificador=fgetl(s); % Hay que encontrar la forma de que se elimine el primer caracter que aparece
datos=zeros(1,cantidad);
tic;a=1;
for i=1:cantidad
    datos(i)=str2double(fscanf(s));    
    t=toc;
    if t>a
        tic;
        disp ('Leyendo datos')
    end    
end

tic;a=1;
while(254~=str2double(fscanf(s)))
    t=toc;
    if t>a
        tic;
        disp ('Esperando a que llegue el fin')
    end
end
% Esto esta bajo prueba
%flushinput(s);  %Vacia el buffer de entrada
%flushoutput(s); %Vacia el buffer de salida
% Esto esta bajo prueba

Dato.cantidad=cantidad;
Dato.identificador=identificador; % Tiene un error: aparece un enter antes del char
Dato.datos=datos;

 



end