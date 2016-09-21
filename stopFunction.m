function stop = stopFunction(accelro,downThr,diffThr,winLength)
%This func decide if the training should stop
stop=false;
arrLen=length(accelro(1,:));

[~, minVal, maxVal]=avgMinMax(accelro,winLength);
diffx=maxVal(1)-minVal(1);
diffy=maxVal(2)-minVal(2);
isStable = abs(diffx) < diffThr && abs(diffy) < diffThr;

if abs(accelro(3,end)) < downThr
    stop=true;
elseif isStable && arrLen > 3
    stop=true;
end


