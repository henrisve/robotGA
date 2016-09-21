function iSelected = TournamentSelect(fitness, tournamentSelectionParameter, tournamentSize)
%This function chooses the best individual

populationSize = size(fitness,1);
iSelect = 1+fix(rand(1,tournamentSize)*(populationSize-1));

[tmp,ind]=sort(fitness(iSelect),'descend');
iSelect=iSelect(ind);

%  if rand is bigger then Pt, select the best, otherwise try again with
%next best etc...
for i=1:tournamentSize 
    if rand < tournamentSelectionParameter
        break;
    end
end
iSelected = iSelect(i);

end


