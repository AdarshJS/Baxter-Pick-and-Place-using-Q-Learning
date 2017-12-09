%My Tic Tac Toe game training phase
clear
clc
Q_Table = rand(4^6,6); %Q table init with random numbers
epsilon = 1;
alpha = 0.3; %Learning Rate
y = 0.2; %discount factor
Episode_counter = 0;
Flag = 0;
Board_Temp = [0 0 0 0 0 0];
for episodes = 1:400000
    
    
    if Flag == 0 %The algo has learned the prev board properly
        [Board, color_3, color_2, color_1 ] = CreateRandomBoard(); %Then assign new random board
        Episode_counter = Episode_counter + 1;
    else
        for i = 1:6
            Board(i) = Board_Temp(i); %Retrain the previous board
        end
    end
        
%     Board = [1 2 1 2 1 3];
%     color_3 = 1;
%     color_2 = 2;
%     color_1 = 3;
    for i = 1:6
        Board_Temp(i) = Board(i);
    end
    Board_Temp
    Board_2 = [0 0 0 0 0 0];
    state = StateCalc(Board); %Initial state
    state_temp = state;
    TerminalState = 0; %Not terminal state
    Winner = 0;
    k = 1;
    
    while(TerminalState == 0 ) %Board 2 is not full
        
        [Q_Table,action] = SelectAction(Q_Table, Board, state_temp, epsilon); %using epsilon greedy actions action stores the index where 1 or 2 has to be entered
        
        Board_2(k) = Board(action); 
        Board(action) = 0; %make that box empty
        
        Next_state = StateCalc(Board);  %Board is updated after the action is taken State is not updated here
        [Reward,Winner,TerminalState] = RewardCalc(Board_2, color_3, color_2, color_1, k);
        B = k;
        k = k+1;
        
        [Q_Table, Q_Best_Next] = QBestValue(Q_Table,Board,Next_state,TerminalState);
        %Q_Table(state,:)
        Q_Table(state_temp,action) = Q_Table(state_temp,action) + alpha*(Reward + y*(Q_Best_Next) - Q_Table(state_temp,action));
        %Q_Table(state,:)
        state_temp = Next_state;
        if TerminalState==1 && Winner == 1
            fprintf('\n Correct Assembly in episode %d\n',Episode_counter);
            Flag = 0;
            Board_2
        elseif TerminalState==1 && Winner == 2
            fprintf('\n Wrong Assembly in episode %d and k = %d\n',Episode_counter,B);
            Flag = 1;
            
        end
    end
end
save('Baxter_Training_2.mat','Q_Table');
            
                
                
                
        
    
    
    

