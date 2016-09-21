function chromosome = Mutate(chromosome,mutationProbability,layers,sigma,weightInit)
%This function takes the chromosome and a probability, and return an
%mutated version of the chromosome.
for u=1:length(layers)-1;
    for j=1:layers(u)
        for k=1:layers(1+u)
            if (rand < mutationProbability(1))
                if (rand < mutationProbability(2))
                    chromosome(j,k,u) = -weightInit + 2*weightInit*rand;
                else
                    chromosome(j,k,u) = chromosome(j,k,u) + normrnd(0,sigma);
                end
            end
        end
    end
end


