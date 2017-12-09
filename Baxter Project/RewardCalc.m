%Reward Scheme - Player1 wins Reward = 1
%Player2 wins Rewards = -1
%If draw reward = 0
function [Reward, Winner, TerminalState] = RewardCalc(Board_2, color_3, color_2, color_1, k)
if k==1 || k==2 || k==3 
    if Board_2(k) == color_3
        Reward = 1;
        Winner = 0; %Game continues
        TerminalState = 0;
    else
        Reward = -1;
        Winner = 2;
        TerminalState = 1;
    end
    
elseif k==4 || k==5
    if Board_2(k) == color_2
        Reward = 1;
        Winner = 0; %Game continues
        TerminalState = 0;
    else
        Reward = -1;
        Winner = 2;
        TerminalState = 1;
    end
    
elseif k==6
    if Board_2(k) == color_1
        Reward = 1;
        Winner = 1;
        TerminalState = 1;
    else
        Reward = -1;
        Winner = 2;
        TerminalState = 1;
    end
end
