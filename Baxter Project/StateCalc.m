%encodes the board config into a stateIndex 
function [state] = StateCalc(Board)
    state = Board*[4^0;4^1;4^2;4^3;4^4;4^5] + 1; %Gives 3^9 values
end
