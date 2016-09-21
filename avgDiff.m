function avgdiff = avgDiff(array,len)
%Takes the Calculate the diff
winLength=min([len size(array,2)])-1;
tmp=diff(array(:,end-winLength:end),1,2);
avgdiff=mean(tmp,2);
avgdiff(isnan(avgdiff))=0;
end
