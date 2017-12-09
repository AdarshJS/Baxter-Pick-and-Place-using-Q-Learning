%This function assigns a value of zero to all invalid actions
function [Q_Table] = UpdateTrueActions_Next(Q_Table, Board, state1)
Unoccupied = find(Board==0);
Q_row_next_state = Q_Table(state1,:);
for i = 1:size(Unoccupied,2) %1 to number of unoccupied boxes
    Q_row_next_state(Unoccupied(i))=0; %The zeros correspond to invalid actions and they must have value zero
end

for i = 1:6
    Q_Table(state1,i) = Q_row_next_state(i);
end
end



