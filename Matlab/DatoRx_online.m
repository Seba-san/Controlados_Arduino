function [Dato]=DatoRx_online(s)
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

Dato=str2double(fscanf(s));
 



end