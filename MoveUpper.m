function motorState = MoveUpper(upperBody,motorState)
%This function will take the output of the NN and update the upper body
%state

 motorState([1,2])=-upperBody(2)-0.4;
 motorState(4)=(-upperBody(1)/2)-.8;
 motorState(3)=(upperBody(1)/2)-.8;
end 