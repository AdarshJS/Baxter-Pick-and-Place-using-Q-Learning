%Player Function I play with the agent
function Player()

example = matfile('States.mat');
state = example.state;

example_3 = matfile('Color_3.mat');
color_3_array = example_3.color_3_array;

example_4 = matfile('Color_2.mat');
color_2_array = example_4.color_2_array;

example_5 = matfile('Color_1.mat');
color_1_array = example_5.color_1_array;

example2 = matfile('Baxter_Training_2.mat');
Q_Table = example2.Q_Table;

Win = 0;
for i = 1:100
    
    [Board, color_3, color_2, color_1 ] = CreateRandomBoard_Simple();
    Board
    Board_2 = [0 0 0 0 0 0];
    
    Index = 357;
    % [Board] = ConvertStatetoBoard(state(Index))
    % color_3 = color_3_array(Index);
    % color_2 = color_2_array(Index);
    % color_1 = color_1_array(Index);
    
    state = StateCalc(Board);
    state_temp = state;
    epsilon = 1;
    
    
    %Q_Table
    Episode_counter = 0;
    TerminalState = 0; %Not terminal state
    Winner = 0;
    k = 1;
    while(TerminalState == 0 ) %Board 2 is not full
        
        [Q_Table,action] = SelectAction(Q_Table, Board, state_temp, epsilon); %using epsilon greedy actions action stores the index where 1 or 2 has to be entered
        
        Board_2(k) = Board(action);
        Board(action) = 0; %make that box empty
        
        Next_state = StateCalc(Board);  %Board is updated after the action is taken State is not updated here
        [Reward,Winner,TerminalState] = RewardCalc(Board_2, color_3, color_2, color_1, k);
        k = k+1;
        B = k-1;
        state_temp = Next_state;
        if TerminalState==1 && Winner == 1
            fprintf('\n Correct Assembly in episode %d\n',Episode_counter);
            Win = Win+1;
            Board_2
        elseif TerminalState==1 && Winner == 2
            fprintf('\n Wrong Assembly in episode %d and k = %d\n',Episode_counter,B);
        end
    end
end
Win
end

  
   
        