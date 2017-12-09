function [Q_Table,action] = SelectAction(Q_Table, Board, state, epsilon)
% tieCounter = 0;
% Best_action = 0;
TrueActions = find(Board~=0); 
Q_Table = UpdateTrueActions(Q_Table, Board, state); %make impossible action values as zeros (empty boxes)
prob =  0.1*randi(9);
if prob < epsilon %Then select Highest value action
    Best_Value = -inf;
    for i = 1:size(TrueActions,2)%1 to number of actions that are possible for this state
        if(Best_Value < Q_Table(state,TrueActions(i)))
            Best_Value = Q_Table(state,TrueActions(i));
            Best_action = TrueActions(i); 
            tieCounter = 1;
            tieActions = Best_action*ones(1,1);
        elseif(Best_Value==Q_Table(state,TrueActions(i)))
            tieCounter = tieCounter + 1;
            tieActions(tieCounter) = TrueActions(i);%Tieactions stores indices of all same value actions 
        end 
    end
    if(tieCounter > 1)
        Best_action = randsample(tieActions, 1, true, (1/tieCounter)*ones(1,tieCounter));%the last argument is the weight given for weighted average it is 1/number of actions with same values.
    end
else %select random action
    Best_action = datasample(TrueActions,1);
end

action = Best_action;
end
        
