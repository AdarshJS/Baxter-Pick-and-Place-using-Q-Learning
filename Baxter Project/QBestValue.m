%This function selects the action that has maximum value in Q table for
%next state
function [Q_Table,Q_Best_Next] = QBestValue(Q_Table,Board,Next_state,TerminalState)
if TerminalState == 1 %if next state is Terminal state, Q(s',a)=0
    Q_Best_Next = 0;
else
    TrueActions_Next = find(Board~=0); %finds indices of non-empty boxes in board. 
    Q_Table = UpdateTrueActions_Next(Q_Table, Board, Next_state);
    Best_Value_Next = -inf;
    for i = 1:size(TrueActions_Next,2)%1 to number of actions that are possible for this state
        if(Best_Value_Next<Q_Table(Next_state,TrueActions_Next(i)))
            Best_Value_Next = Q_Table(Next_state,TrueActions_Next(i));
        end 
    end
    Q_Best_Next = Best_Value_Next;
end
end