try
    TerminateRobotInterface(robot);
catch
end
try
    fclose(s);
    delete(s);
catch
end
clear all
%%Parameters


%%Accelerometer
upThr=30; %start/stop when x and y is above this
downThr=50; %stop when z is below this
diffThr=10; %when diff is less, it's stable
winLength=20; %length to test
maxAcc=[0,0];

%GA
populationSize = 6;
crossoverProbability = 0.25;%
mutationProbability = [0.05 0.05];%second is full or creep
mutationSigma=.2;
tournamentSelectionParameter = 0.75;
tournamentSize = 2;
numberOfGenerations = 40;
eliteCopies = 1;
lenUpChrome = 0;
lenLoChrome = 0;
otherLegPercent = 0;
cSplitParm = [lenUpChrome,lenLoChrome,otherLegPercent];
maxtime=150;
trainRuns=2;

%neuralNet
hiddenlayers=[6]; % use array to play with multilayer:)
inputs=5;
outputs=3; % Two for upper body, two for lower body %
weightInit=1;
beta=1;

%other
plotingTraing=true;
plotingValid=true;
useRobot=1;

%%%% variables and init
%Robot
if(useRobot)
    robot = CreateRobotInterface(13,5);
    for iServo = 1:robot.NUMBER_OF_SERVOS
        calllib('dynamixel','dxl_write_word', iServo, robot.ADRESS_ENABLE, 0)
    end
    disp(sprintf('Set a desired start position and press any key.'));
    waitforbuttonpress;
    startposition =  [-0.36,-0.34,-0.52,-0.54,-0.043,-0.00,-0.00,0.16,0.97,1.30,0.77,0.77,1.07,1.09,0.54,0.58,-0.04,0.11];%GetServoState(robot);
    prevMotorState = startposition;
    toBeSet=ones(1, robot.NUMBER_OF_SERVOS);
    s=robot.globalState;
end

%NN
layers=[inputs hiddenlayers outputs];

%ga
fitness = zeros(populationSize,1);

%%Accelrometer
accelro=[;];

%%Init the chromosome
population = [];
for iPop = 1:populationSize
    Wik1=zeros(max([inputs hiddenlayers]),max([hiddenlayers outputs]),length(layers)-1);%migth be wrong!  why not a cell or struct??
    for u=1:length(layers)-1;
        for j=1:layers(u)
            for k=1:layers(1+u)
                Wik1(j,k,u) = -(0.1) + 2*0.1*rand;%-weightInit + 2*weightInit*rand;
            end
        end
    end
    Wik1(1,1,1)= 0.5;
    Wik1(3,1,1)= -0.32;    
    Wik1(2,3,1)= 0.5;
    Wik1(4,3,1)=-0.32;   
    Wik1(1,1,2)=1;
    Wik1(3,2,2)=1;
    
    Wik2=zeros(max([inputs hiddenlayers]),max([hiddenlayers outputs]),length(layers)-1);
    for u=1:length(layers)-1;
        for j=1:layers(u)
            for k=1:layers(1+u)
                Wik2(j,k,u) = -(0.1) + 2*0.1*rand;
            end
        end
    end
    
    population = [population; struct('upperChromosome',Wik1,'lowerChromosome'...
        ,Wik2,'otherSidePercent',2*rand)];
end

%fitness for validation???
maximumFitnessValid = 0.0;

disp('init done');
maximumFitness = zeros(numberOfGenerations,1);
for iGeneration = 1:numberOfGenerations
    pause(2);
    bestIndividualIndex = 0;
    for iPop =1:populationSize
        upperChromosome = population(iPop).upperChromosome;
        lowerChromosome = population(iPop).lowerChromosome;
        fitnessTemp=zeros(trainRuns,1);
        for iRun=1:trainRuns;
            accelro=[];
            for iServo = 1:robot.NUMBER_OF_SERVOS
                temp(iServo) = calllib('dynamixel','dxl_read_word', iServo, 43);
            end
            %%Due to dynamixel fail sometimes, we need to reinit them
            while sum(abs(diff(temp))) < 1 || max(temp) > 55
                disp('temperature error, ')
                for iServo = 1:robot.NUMBER_OF_SERVOS
                    calllib('dynamixel','dxl_write_word', iServo, robot.ADRESS_ENABLE, 0)
                end
                pause(5);
                for iServo = 1:robot.NUMBER_OF_SERVOS
                    temp(iServo) = calllib('dynamixel','dxl_read_word', iServo, 43) + 2;
                end
                disp(temp);
                TerminateRobotInterface(robot);
                pause(5);
                robot = CreateRobotInterface(13,5);
                s=robot.globalState;
                pause(1);
            end
            disp(temp);
            pause(.3)
            SetServoState(robot, startposition);
            pause(.3)
            disp('Running...');
            motorState = GetServoState(robot);
            motorState(1)=-.7;
            motorState(2)=-.7;
            SetServoState(robot, motorState);
            for iLoop=1:maxtime
                %Read accelrometer
                while s.BytesAvailable > 0
                    fscanf(s,'%s');
                end
                reads=fscanf(s,'%i,%i,%i');                
                if size(accelro,1)==0
                    accelro=[reads(1);reads(2);reads(3)];
                else
                    accelro=[accelro(1,:) reads(1);accelro(2,:) reads(2);accelro(3,:) reads(3)];
                end

                %Check if we need to stop
                if stopFunction(accelro,downThr,diffThr,winLength)
                    disp('Run Stopped... ');
                    for iServo = 1:robot.NUMBER_OF_SERVOS
                        calllib('dynamixel','dxl_write_word', iServo, 25, 0)
                    end
                    break;
                end
                
                accelroSlowLow = avgMinMax(accelro,50);
                accelroSlow = avgMinMax(accelro,4);
                accelSlope = avgDiff(accelro,7);
                
                [~, ind(1)]=max([abs(maxAcc(1)) abs(accelro(1,end))]);
                [~, ind(2)]=max([abs(maxAcc(2)) abs(accelro(2,end))]);
                if ind(1) == 1
                    maxAcc(1)=maxAcc(1)*0.95;
                else
                    maxAcc(1)=accelroSlow(1,end);
                end
                if ind(2) == 1
                    maxAcc(2)=maxAcc(2)*0.95;
                else
                    maxAcc(2)=accelroSlow(2,end);
                end
                
                upperBody = NeuralNet([1 maxAcc(1)/100 maxAcc(2)/100 accelSlope(1)/100 accelSlope(2)/100],upperChromosome,layers,beta);
                lowerBody = NeuralNet([1 accelroSlowLow(1)/100 accelroSlowLow(2)/100 accelSlope(1)/100 accelSlope(2)/100],lowerChromosome,layers,beta);
                
                if(useRobot)
                    motorState = GetServoState(robot,toBeSet,prevMotorState);
                    motorState=MoveUpper(upperBody,motorState);
                    motorState=MoveLower(lowerBody,motorState,population(iPop).otherSidePercent,sideLeft);
                    
                    %after both upper and lower is set, move motors
                    toBeSet = abs(prevMotorState - motorState) > 0.01;
                    prevMotorState=motorState; 
                    SetServoState(robot, motorState, toBeSet);                    
                end
                
                if iLoop==1
                    plotArray=[maxAcc(1) maxAcc(2) accelSlope(1) accelSlope(2) upperBody(1) upperBody(2)];
                else
                    plotArray=[plotArray;maxAcc(1) maxAcc(2) accelSlope(1) accelSlope(2) upperBody(1) upperBody(2)];
                end
            end   
            fitnessTemp(iRun) = fitness_function(accelro,downThr);
            if fitnessTemp(iRun) < 1 %% it fell and not standing
                for iServo = 1:robot.NUMBER_OF_SERVOS
                    calllib('dynamixel','dxl_write_word', iServo, robot.ADRESS_ENABLE, 0)
                end
            end
            clf
            subplot(4,1,1)
            
            plot(plotArray(:,1),'r')
            hold on
            plot(plotArray(:,2),'g')
            subplot(4,1,2)
            
            plot(plotArray(:,3),'r')
            hold on
            plot(plotArray(:,4),'g')
            subplot(4,1,3)
           
            plot(plotArray(:,5),'r')
            hold on
            plot(plotArray(:,6),'g')
            
            subplot(4,1,4)
            plot(maximumFitness,'g')
        end
        
        fitness(iPop)=mean(fitnessTemp); % TRY DIFFERENT!
        if (fitness(iPop) > maximumFitness(iGeneration))
            maximumFitness(iGeneration) = fitness(iPop);
            bestIndividualIndex = iPop;
            bestLowerIndividual = population(iPop).lowerChromosome;
            bestUpperIndividual = population(iPop).upperChromosome;
        end
    end
    %% Evolve population.
    tempPopulation = population;
    for iPop = 1:2:populationSize
        %Upper body
        i1 = TournamentSelect(fitness,tournamentSelectionParameter,tournamentSize);
        i2 = TournamentSelect(fitness,tournamentSelectionParameter,tournamentSize);
        chromosome1 = population(i1).upperChromosome;
        chromosome2 = population(i2).upperChromosome;
        if (rand < crossoverProbability)
            [chromosome1, chromosome2]= CrossNN(chromosome1,chromosome2,layers);
        end
        tempPopulation(iPop).upperChromosome = Mutate(chromosome1,mutationProbability,layers,mutationSigma,weightInit);
        tempPopulation(iPop+1).upperChromosome = Mutate(chromosome2,mutationProbability,layers,mutationSigma,weightInit);
        
%         %Lower body
        i1 = TournamentSelect(fitness,tournamentSelectionParameter,tournamentSize);
        i2 = TournamentSelect(fitness,tournamentSelectionParameter,tournamentSize);
        chromosome1 = population(i1).lowerChromosome;
        chromosome2 = population(i2).lowerChromosome;
        if (rand < crossoverProbability)   %No crossover, maybe later
            [chromosome1, chromosome2]= CrossNN(chromosome1,chromosome2,layers);
        end
        tempPopulation(iPop).lowerChromosome = Mutate(chromosome1,mutationProbability,layers,mutationSigma,weightInit);
        tempPopulation(iPop+1).lowerChromosome = Mutate(chromosome2,mutationProbability,layers,mutationSigma,weightInit);
    end
    
    population = InsertBestIndividual(tempPopulation, bestUpperIndividual, bestLowerIndividual, eliteCopies);
    population.upperChromosome
end

fclose(s);
delete(s);
try
    fclose(s);
    delete(s);
    TerminateRobotInterface(robot);
catch
end