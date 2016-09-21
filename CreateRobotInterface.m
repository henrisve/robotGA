%http://support.robotis.com/en/software/dynamixel_sdk/api_reference.htm
%http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm

function robot = CreateRobotInterface(comServo, comGlobal)
% comServo  = 5/16 stationary/laptop
% comGlobal = 4/?  stationary/laptop
    MAX_VALUE = 2^10 - 1;

    BAUDRATE_NUMBER = 1;
    NUMBER_OF_SERVOS = 18;

    % Given constants by manufacturer
    ADRESS_MAX_TORQUE       = 34; %14,34 % 2^10-1 is 100% of max.
    
    ADRESS_GOAL_POSITION    = 30;
    ADRESS_PRESENT_POSITION = 36;
    
    ADRESS_GOAL_SPEED       = 32; 
    ADRESS_PRESENT_SPEED    = 38;
    ADRESS_ENABLE           = 24;

    %Initialize Servos
    loadlibrary('dynamixel','dynamixel.h')
    response = calllib('dynamixel','dxl_initialize',comServo,BAUDRATE_NUMBER);
    
    if response ~= 1
        robot = NaN;
        disp('ERROR: Failed to open USB2Dynamixel!')
    else
        %Initialize Global Parameters
        [s,flag] = setupSerial(['COM' num2str(comGlobal)]);
        
        % Boundarys
        % SERVO    1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18
        MIN_ANG = [0   0   250 250 200 200 200 200 350 350 100 100 50  50  200 200 350 350];
        MAX_ANG = [800 800 750 750 800 800 500 500 500 500 600 600 500 500 590 590 650 650];
        
        maxLoad  = 0.5; %times max
        defSpeed = 0.5; % default speed
        for iServo = 1:NUMBER_OF_SERVOS
            calllib('dynamixel','dxl_write_word', iServo, ADRESS_MAX_TORQUE, mean(maxLoad*MAX_VALUE))
            calllib('dynamixel','dxl_write_word', iServo, ADRESS_GOAL_SPEED, mean(defSpeed*MAX_VALUE))
        end
        
        robot = struct( 'NUMBER_OF_SERVOS',         NUMBER_OF_SERVOS,...
                        'ADRESS_MAX_TORQUE',        ADRESS_MAX_TORQUE,...
                        'ADRESS_GOAL_POSITION',     ADRESS_GOAL_POSITION,...
                        'ADRESS_PRESENT_POSITION',  ADRESS_PRESENT_POSITION,...
                        'ADRESS_GOAL_SPEED',        ADRESS_GOAL_SPEED,...
                        'ADRESS_ENABLE',            ADRESS_ENABLE,...
                        'ADRESS_PRESENT_SPEED',     ADRESS_PRESENT_SPEED,...
                        'MIN_ANG',                  MIN_ANG,...
                        'MAX_ANG',                  MAX_ANG,...
                        'globalState',              s);
    end
end