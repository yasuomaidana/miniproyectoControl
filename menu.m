clear
clc
close all
while ~xor(op~=1,op~=2)
    op=input('Desea : \n    simular el carro(1) \n    cargar las graficas de una simulacion previa(2) \n Opcion :');
    if ~xor(op~=1,op~=2)
        disp('Seleccione una opcion valida')
    end
end
switch op
    case 1
        
    case 2
end