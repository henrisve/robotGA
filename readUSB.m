try
    fclose(s);
    delete(s);
catch
end
clear all
downThr=50;
diffThr=20;
upThr=20;

[s,flag] = setupSerial('COM5');
accelro=[];
maccelro=[];
maxAcc=[0,0];

fscanf(s,'%i,%i,%i')
for i = 1:5000;
    winLength=20;
    
    while s.BytesAvailable > 0
%         s.BytesAvailable
%         pause(0.5);
%         s.BytesAvailable
        fscanf(s,'%s');
    end
    
    reads=fscanf(s,'%i,%i,%i');
    if size(accelro,1)==0
        accelro=[reads(1);reads(2);reads(3)];
    else
        accelro=[accelro(1,:) reads(1);accelro(2,:) reads(2);accelro(3,:) reads(3)];
    end
    [tmp ind(1)]=max([abs(maxAcc(1)) abs(accelro(1,end))]);
    [tmp ind(2)]=max([abs(maxAcc(2)) abs(accelro(2,end))]);
    
    if ind(1) == 1
        maxAcc(1)=maxAcc(1)*0.9;
    else
        maxAcc(1)=accelro(1,end);
    end
    if ind(2) == 1
        maxAcc(2)=maxAcc(2)*0.9;
    else
        maxAcc(2)=accelro(2,end);
    end
    
   
    if size(maccelro,1)==0
        maccelro=[maxAcc(1);maxAcc(2)];
    else
        maccelro=[maccelro(1,:) maxAcc(1);maccelro(2,:) maxAcc(2)];
    end
    
    
     subplot(4,1,1);
     plot(accelro(1,:)');
     subplot(4,1,3);
     plot(accelro(2,:)');
     
     
     subplot(4,1,2);
     plot(maccelro(1,:)');
     subplot(4,1,4);
     plot(maccelro(2,:)');
    
    
    drawnow
% % % % %     
% % % % %     
% % % % %     stop=false;
% % % % %     winLength=min([winLength length(accelro(1,:))])-1;
% % % % %     diffx=max(accelro(1,end-winLength:end))-min(accelro(1,end-winLength:end));
% % % % %     diffy=max(accelro(2,end-winLength:end))-min(accelro(2,end-winLength:end));
% % % % %     isStable = abs(diffx) < diffThr && abs(diffy) < diffThr;
% % % % %     
% % % % %     
% % % % %     if abs(accelro(3,end)) < downThr
% % % % %         stop=1;
% % % % %     elseif isStable && (abs(accelro(1,end)) < upThr && abs(accelro(2,end)) < upThr)
% % % % %         stop=2;
% % % % %     else
% % % % %         stop=0;
% % % % %     end
% % % % %     disp(sprintf('stop:%d stable:%d x:%d y:%d z:%d diffx:%d diffy:%d bytes:%d',stop,isStable,accelro(1,end),accelro(2,end),accelro(3,end),diffx,diffy,s.BytesAvailable))
end

fclose(s);
delete(s);

try
    fclose(s);
    delete(s);
catch
end
