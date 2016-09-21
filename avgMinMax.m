function [avgVal,minVal,maxVal] = avgMinMax(array,len)
%Takes the  arages,min and max of the l last values
winLength=min([len size(array,2)])-1;
avgVal=mean(array(:,end-winLength:end),2);
minVal=min(array(:,end-winLength:end),[],2);
maxVal=max(array(:,end-winLength:end),[],2);
end

