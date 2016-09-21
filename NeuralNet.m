function output = NeuralNet(input,Wik,layers,beta)
%NeuralNet This function takes an input layer, w and nr of neurons in each 
%hidden layer and compute the output layer
%Hiddenlayers is an array, with any amounts of layers/neurons per layer

if size(layers(1)) ~= size(input)
   error('input size must be same in "input" and first element of "layers"') 
end

layerValue(:,1) = input;
for u=1:length(layers)-1;
    for k=1:layers(u+1)
        b=0;
        for j=1:layers(u)
            b=b+Wik(j,k,u)*layerValue(j,u); 
        end
        layerValue(k,u+1) = (sigmf(beta*(b) , [3,0])*2)-1;
    end
end

output=layerValue(1:layers(end),end);

