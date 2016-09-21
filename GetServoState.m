function states = GetServoState(robot,toBeRead,oldState)
if nargin < 2
    toBeRead=ones(1, robot.NUMBER_OF_SERVOS);
end

    states = zeros(1, robot.NUMBER_OF_SERVOS);
    
    for iServo = 1:robot.NUMBER_OF_SERVOS
        if toBeRead(iServo)
            states(iServo) = calllib('dynamixel','dxl_read_word', iServo, robot.ADRESS_PRESENT_POSITION);

            if ~mod(iServo,2)
                states(iServo) = (2^10 - 1) - states(iServo);
            end

            span = robot.MAX_ANG(iServo) - robot.MIN_ANG(iServo);
            states(iServo) = (states(iServo) - robot.MIN_ANG(iServo))/span;
            states(iServo) = 2*states(iServo) - 1;
        else
            states(iServo) = oldState(iServo);
        end
    end
end