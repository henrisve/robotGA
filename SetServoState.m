function SetServoState(robot, states, toBeSet)
    %%This function moves servos on robot to desired state
    if nargin < 3 %% Should make it back compatible
        toBeSet = ones(robot.NUMBER_OF_SERVOS,1);
    end
    for iServo = 1:robot.NUMBER_OF_SERVOS
        if toBeSet(iServo)
            if states(iServo) < -1
                states(iServo) = -1;
            elseif states(iServo) > 1
                states(iServo) = 1;
            end

            state = (states(iServo) + 1)/2;
            span  = (robot.MAX_ANG(iServo) - robot.MIN_ANG(iServo));
            state = mean(state*span + robot.MIN_ANG(iServo));

            if ~mod(iServo,2) 
                state = (2^10 - 1) - state;
            end
           
            calllib('dynamixel','dxl_write_word', iServo, robot.ADRESS_GOAL_POSITION, state)
        end
    end
end