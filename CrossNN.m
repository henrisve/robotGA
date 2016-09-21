function [newchromosome1,newchromosome2] = CrossNN(chromosome1,chromosome2,layers)
%This funcntion will randomly cross chromosomes for NN
%It goes trough each layer and swap once in each dimmension
for u = 1:length(layers)-1
    crossoverPoint = [randi([0 layers(u)]) randi([0 layers(u+1)])];
    for i = 1:layers(u)
        for j = 1:layers(u+1)  
            if xor(crossoverPoint(1) > i,crossoverPoint(2) > j);
                test(i,j,u)=1;
                newchromosome1(i,j,u)=chromosome2(i,j,u);     
                newchromosome2(i,j,u)=chromosome1(i,j,u);  
            else
                test(i,j,u)=0;
                newchromosome1(i,j,u)=chromosome1(i,j,u);     
                newchromosome2(i,j,u)=chromosome2(i,j,u);  
            end
        end
    end
end
