clc
clear
%% Inicialización
s=1;        % scale factor for the robot in coppelia
nLinVel=0.20;       % normal linear velocity for the wheels

disp('Program started');
%% Conexión con simulador
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); % localhost
%% Verificación conexión con simulador
if (clientID>-1)
    disp('Connected to remote API server');    
else
    disp('Failed connecting to remote API server');
end

%% Inicio de simulación en conjunto con matlab
if clientID>-1      % Only if the connection was achieved
    %% Generación de "objetos de control" 'handlers'
    %Now try to retrieve handlers in a blocking fashion 
    [joint_handler, sensor_handler, base_handler] = get_handlers(...
        sim, clientID);
    disp('Joint handles acquired')     
    pause(2);
    
    %% Control Loop
    % test if sensors can be read
    disp('leyendo sensores')
    n=1;
    base_pos=zeros(1,3);
    
    %% Lectura de sensores por primera vez
    [sensors, base_pos(n,:)] = get_data_sensor(sim, clientID,...
        sensor_handler, base_handler,1);
    %% Inicialización de clock
    t = clock;
    startTime = t(6);
    currentTime = t(6);
    T_final = 60;       % simulation time
    T = 0.025;           % sample time
    T0=0;
    
    %% Inicialización de variables de control
    e=zeros(2,3);     % error signal
    u=zeros(2,3);
    % nLinVel es el limitador de velocidad ya que en la vida real nuestro
    % carro no podría ir más allá de una velocidad límite por el motor
    nLinVel;
    %Realiza la simulacion
    base_pos=simula(currentTime,nLinVel,u,e,startTime,T_final,T0,T,sim,clientID,sensor_handler,base_handler,joint_handler);
    %Detiene los motores
    send_motors(sim,clientID, joint_handler, [0,0]);
    %% Make sure that the last command sent out had time to arrive.
    sim.simxGetPingTime(clientID);
    % Now close the connection to CoppeliaSim:    
    sim.simxFinish(clientID);
end
sim.delete(); % call the destructor!
%save('camino.mat','base_pos')
disp('Program ended');
y=base_pos(4:end,2);
x=base_pos(4:end,1);
bezierPlotter('camino.csv')
hold on
plot(x,y,'r')
legend('Camino propuesto','Trayectoria reccorida')
title('Camino propuesto vs recorrido')
basepos=[x,y];
save('trayectoria.mat','basepos');
%{
figure(1)
plot(base_pos);
figure(2)
y=base_pos(:,1);
x=base_pos(:,2);
m(1)=0;
for (i=1:size(x,1)-1)
    dx=x(i+1)-x(i);
    dy=y(i+1)-y(i);
    m(i+1)=dx/dy;
end
plot(x,y)
hold on
%}

%% Realiza la simulación
function base_pos=simula(currentTime,nLinVel,u,e,startTime,T_final,T0,T,sim,clientID,sensor_handler,base_handler,joint_handler)
    prev=[1,1];
    n=1;
    while (currentTime-startTime < T_final&&n<2500)
        if currentTime-T0>T
            t=clock;
            T0=t(6);
            if(T0>59)
                T0=0;
            end
            %Lee los datos del sensor y posicion
            [sensors, base_pos(n,:)] = get_data_sensor(sim, clientID,...
                sensor_handler, base_handler, 0);             
            %disp('Sensores')
            %disp(sensors.')
            
            %Obtiene el tipo de movimiento a realizar
            [setL,setR]= movimiento(sensors,prev,nLinVel);
            %Define el nuevo setpoint de cada rueda
            set = [setL,setR];
            prev = set;
            
            %%disp('Set')
            %%disp(set)
            %Calcula las variables de manipulacion con un dead beat
            [uk,ek] = deadCo(u,e,set,nLinVel);
            %disp('Uk')
            %disp(uk.')
            % send data
            send_motors(sim,clientID, joint_handler, uk);
            % rotate samples
            u = rotation(u);
            e = rotation(e);
            u(:,3)=uk;
            e(:,3)=ek;
            n=n+1;
        end
        % timer 
        t=clock;
        currentTime=t(6);
    end
end
%% API functions
%% Envía la información a los motores
%{
A traves de los objetos joint_handlers envía la velocidad a la que se
moverán los joints, dicha velocidad lineal es transformada en angular
dividiendo la velocidad entre el radio y la escala
%}
function send_motors(sim,clientID, joint_handlers, velocity)
    s=1;        % scale factor
    wheelRadius=0.027;
    interWheelDistance=0.119;
    joint_dyn_left = joint_handlers{1};
    joint_dyn_right = joint_handlers{2};
    [returnCode]=sim.simxSetJointTargetVelocity(clientID, joint_dyn_left...
        ,velocity(1)/(s*wheelRadius), sim.simx_opmode_oneshot);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID, joint_dyn_right...
        ,velocity(2)/(s*wheelRadius), sim.simx_opmode_oneshot);
    sim.simxAddStatusbarMessage(clientID,['recieved: ' num2str(velocity(1)) ' , ' num2str(velocity(2))]...
            ,sim.simx_opmode_oneshot);
end
%% Genera objetos para controlar
%{
Genera los objetos con los cuales se puede leer la información del estado
del rebot, regresa los datos en la siguiente estructura
joints{izquierda,derecha}
sensors{izquierda,centro,derecha}
base
%}
function [joints, sensors, base] = get_handlers(sim, clientID)
    [returnCode, joint_dyn_left]=sim.simxGetObjectHandle(clientID...
        ,'DynamicLeftJoint', sim.simx_opmode_blocking);
    [returnCode, joint_dyn_right]=sim.simxGetObjectHandle(clientID...
        ,'DynamicRightJoint', sim.simx_opmode_blocking); 
    [returnCode, leftSensor]=sim.simxGetObjectHandle(clientID...
        ,'LeftSensor', sim.simx_opmode_blocking);
    [returnCode, middleSensor]=sim.simxGetObjectHandle(clientID...
        ,'MiddleSensor', sim.simx_opmode_blocking);
    [returnCode, rightSensor]=sim.simxGetObjectHandle(clientID...
        ,'RightSensor', sim.simx_opmode_blocking);
    [returnCode, base]=sim.simxGetObjectHandle(clientID...
        ,'LineTracerBase', sim.simx_opmode_blocking);
    sim.simxAddStatusbarMessage(clientID,'Handles recieved'...
            ,sim.simx_opmode_oneshot);
    joints = {joint_dyn_left, joint_dyn_right};
    sensors = {leftSensor, middleSensor, rightSensor};
end

%% Lee los datos
%{
A traves de los objetos handlers lee los datos del robot regresa los
siguientes datos
sensors{izquierda,centro,derecha} de tipo normalmente cerrados
base{y,x,altura}
%}
function [sensors, base_pos] = get_data_sensor(sim, clientID, handlers,...
    base_handler, first)
    if first
        mode=sim.simx_opmode_streaming;  % first time executed
    else
        mode=sim.simx_opmode_buffer;
    end
    sensors = [0; 0; 0];
    leftSensor = handlers{1};
    middleSensor = handlers{2};
    rightSensor = handlers{3};
    [returnCode, sensors(1)]=sim.simxReadVisionSensor(clientID,...
        leftSensor, mode);
    [returnCode, sensors(2)]=sim.simxReadVisionSensor(clientID,...
        middleSensor, mode);
    [returnCode, sensors(3)]=sim.simxReadVisionSensor(clientID,...
        rightSensor, mode);
    [returnCode, base_pos]=sim.simxGetObjectPosition(clientID,...
        base_handler, -1, mode);
end

%% Selecciona el tipo de movimiento dependiendo de la señal de los sensores
%{
    Regresa el setpoint de la rueda izquierda y derecha dependiendo del
    caso
%}
function [setL,setR]=movimiento(sensor,prev,nLinVel)
    
    sen=not(sensor);%para trabajar en logica positiva
    %disp("Read Sensor")
    %disp(sen.')
    L=sen(1);%señal izquierda
    C=sen(2);%señal centro
    R=sen(3);%señal derecha
    %disp("L C R")
    %disp([L,C,R])
    keySet = {'Izquierda','Centro','Derecha'};
    valueSet = {[-nLinVel*.05,nLinVel*.75],[1,1],[nLinVel*.75,-nLinVel*.05]};
    Move = containers.Map(keySet,valueSet);
    if not(L||C||R)|| L&&C&&R
        setL = prev(1);
        setR = prev(2);
        %disp('Default')
    else
        if(L)
            Ret= Move('Izquierda');
            %disp('Izquierda')
        end
        if(C)
            Ret= Move('Centro');
            %disp('Centro')
        end
        if(R)
            Ret= Move('Derecha');
            %disp('Derecha')
        end
        setL=Ret(1);
        setR=Ret(2);
    end
    
end
%% Rotación de señal
%{
Rota la señal de tal forma que el más viejo queda en la posición 1 y la mas
nueva queda en la posicion 1
%}
function sigp = rotation(signal)
    [m,n] = size(signal);
    sigp = zeros(m,n);
    sigp(:,1:end-1) = signal(:,2:end);
end