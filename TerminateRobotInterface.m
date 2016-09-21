function TerminateRobotInterface(robot)

    calllib('dynamixel','dxl_terminate');
    unloadlibrary('dynamixel');
    
    fclose(robot.globalState);
    delete(robot.globalState);
end