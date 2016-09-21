function population = InsertBestIndividual(population, bestUpperIndividual,bestLowerIndividual, bestCopies)
%Take the best and insert it x times first in the vector
    for i=1:bestCopies
        population(i).lowerChromosome = bestLowerIndividual;
        population(i).upperChromosome = bestUpperIndividual;
    end
end

