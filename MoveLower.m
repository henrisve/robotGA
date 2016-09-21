function motorState = MoveLower(lowerBody,motorState,otherSide,sideLeft)
%This function will take the output of the NN and update the lower body
%state
%instead of legs we lean the body forward and back

motorState(11)=(lowerBody(1))+0.77;
motorState(12)=(lowerBody(1))+0.77;
