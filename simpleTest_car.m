clc
s=1;        % scale factor for the robot in coppelia
nLinVel=0.20;       % normal linear velocity for the wheels
disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); % localhost
if (clientID>-1)
    disp('Connected to remote API server');    
else
    disp('Failed connecting to remote API server');
end
    
if clientID>-1      % Only if the connection was achieved
    %%  Now try to retrieve handlers in a blocking fashion 
    [joint_handler, sensor_handler, base_handler] = get_handlers(...
        sim, clientID);
    disp('Joint handles acquired')     
    pause(2);
    
    %% Control Loop
    % initial values for control
    LinVelLeft = nLinVel*s;
    LinVelRight = nLinVel*s;
    Vel_ini=[LinVelLeft,LinVelRight];
    % test if sensors can be read
    disp('leyendo sensores')
    n=1;
    base_pos=zeros(1,3);
    [sensors, base_pos(n,:)] = get_data_sensor(sim, clientID,...
        sensor_handler, base_handler,1);
    t = clock;
%         disp(base_pos)
    startTime = t(6);
    currentTime = t(6);
    T_final = 30;       % simulation time
    T = 0.35;           % sample time 
    ek = [0, 0, 0];     % error signal
    Kp = 0.017;         % PID constants
    I = 0.35;
    Td = ones(1,1)*0.01;
    Ti = Kp./I ;
    Tt = (Ti.*Td);
    satlim = [-Vel_ini(1) Vel_ini(1);-Vel_ini(2) Vel_ini(2)]; % control limits 
    u = ones(2,2).*Vel_ini(1)/2;     % Initial condition for u
    up = u;
    T0=0;
        
    while (currentTime-startTime < T_final)
        if currentTime-T0>T
            t=clock;
            T0=t(6);
            % retrieve data from sensors
            [sensors, base_pos(n,:)] = get_data_sensor(sim, clientID,...
                sensor_handler, base_handler, 0);             
            % data proccessing
            k = 3;
            ek(:,k) = decision2(sensors);
            [up(:,k),u(:,k)] = pifunc(ek...
                                   , u(:,k-1), up(:,k-1),...
                                   Kp, Ti, Td, Tt, T, satlim);
            % send data
            send_motors(sim,clientID, joint_handler, u(:,k));
            % rotate samples
            u = rotation(u);
            up = rotation(up);
            ek = rotation(ek);
            n=n+1;
        end
            % timer 
            t=clock;
            currentTime=t(6);
    end
        send_motors(sim,clientID, joint_handler, [0,0]);
        %% Make sure that the last command sent out had time to arrive.
        sim.simxGetPingTime(clientID);
        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
end
sim.delete(); % call the destructor!
save('camino.mat','base_pos')
disp('Program ended');

%% API functions
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

%% Control functions
function [error]=decision2(sensor)
    error=0;
    if (sensor(1)==1 && sensor(2)==1 && sensor(3)==0)
        error=2;
    elseif (sensor(1)==1 && sensor(2)==0 && sensor(3)==0)
        error=1;
    elseif (sensor(1)==1 && sensor(2)==0 && sensor(3)==1)
        error=0;
    elseif (sensor(1)==0 && sensor(2)==0 && sensor(3)==1)
        error=-1;
    elseif (sensor(1)==0 && sensor(2)==1 && sensor(3)==1)
        error=-2;
    elseif (sensor(1)==1 && sensor(2)==1 && sensor(3)==1)
        % search for line
        error=-2;
    elseif (sensor(1)==0 && sensor(2)==0 && sensor(3)==0)
        % search for line
        error=-2;
    elseif (sensor(1)==0 && sensor(2)==1 && sensor(3)==0)
        % search for line
        error=-2;  
    end
end

function sigp = rotation(signal)
    [m,n] = size(signal);
    sigp = zeros(m,n);
    sigp(:,1:end-1) = signal(:,2:end);
end