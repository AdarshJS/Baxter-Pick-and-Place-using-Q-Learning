function[Board] = ConvertStatetoBoard(state)
%state = 1694;
Board = [0,0,0,0,0,0];
Board_Powers = [4^0,4^1,4^2,4^3,4^4,4^5];
reverseCounter = 6;
state_temp = state;
state_temp = state_temp - 1;
while(reverseCounter >= 1)
    if(state_temp >= Board_Powers(reverseCounter))
        Board(reverseCounter) = floor(state_temp/Board_Powers(reverseCounter));
        state_temp = mod(state_temp,Board_Powers(reverseCounter));
        reverseCounter = reverseCounter - 1;
    else
        reverseCounter = reverseCounter - 1;
    end
end
% Board
% Actual_state = StateCalc(Board);
% if (state==Actual_state)
%      fprintf('Correct');
% end
end
        
    
    