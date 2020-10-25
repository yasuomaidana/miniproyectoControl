clear
clc
close all
op=0;
while ~xor(op~=1,op~=2)
    op=input('Desea : \n    simular el carro(1) \n    cargar las graficas de una simulacion previa(2) \n Opcion :');
    if ~xor(op~=1,op~=2)
        disp('Seleccione una opcion valida')
    end
end

switch op
    case 1
        deadProofController();
    case 2
        
        bezierPlotter('camino.csv')
        hold on
        load('trayectoria','basepos')
        x=basepos(:,1);
        y=basepos(:,2);
        plot(x,y,'r')
        legend('Camino propuesto','Trayectoria reccorida')
        title('Camino propuesto vs recorrido')
        basepos=[x,y];
end