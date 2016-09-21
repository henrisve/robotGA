function fitness = fitness_function(array,downThr)
    
   d=(abs(array(1,end))+abs(array(2,end)))/2;
    t = (size(array,2)+d)/100;
    
    if abs(array(3,end)) < downThr
        robot_stance = 0;
    else
        robot_stance = 1;
    end
        
      
    if robot_stance == 0 
        fitness = 1/(1+exp(-t));
    elseif robot_stance == 1
        fitness = 1+exp(-t);
    end
        

end