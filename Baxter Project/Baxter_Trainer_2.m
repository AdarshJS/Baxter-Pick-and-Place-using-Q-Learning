%My Tic Tac Toe game training phase
clear
clc
%Q_Table = rand(4^6,6); %Q table init with random numbers
example = matfile('Baxter_Training_2.mat');
Q_Table = example.Q_Table;
epsilon = 1;
alpha = 0.3; %Learning Rate
y = 0.2; %discount factor
Episode_counter = 0;
Board_2 = [0 0 0 0 0 0];

example = matfile('States.mat');
state = example.state;
%size_state = size(state)

example = matfile('Color_3.mat');
color_3_array = example.color_3_array;

example = matfile('Color_2.mat');
color_2_array = example.color_2_array;

example = matfile('Color_1.mat');
color_1_array = example.color_1_array;
m = 1;
for i = 1:60
    state_temp = state(i);
    state_2 = state_temp;
    color_3 = color_3_array(i);
    color_2 = color_2_array(i);
    color_1 = color_1_array(i);
    [Board] = ConvertStatetoBoard(state_2);
    TerminalState = 0; %Not terminal state
    Winner = 0;
    k = 1;
    Flag = 0;
    
    for j=1:6
        Board_Temp(j) = Board(j);%For initial case
    end
    
    while(Flag ~= 1) %Training not over
        if (Flag == 2) %If wrong assembly in the previous case
            Flag = 0;
            state_2 = state_temp; %original state
            for j=1:6
                Board_Temp(j) = Board(j);
            end
            %fprintf('entered the loop');
        end
        %Board_Temp
        [Q_Table,action] = SelectAction(Q_Table, Board_Temp, state_2, epsilon); %using epsilon greedy actions action stores the index where 1 or 2 has to be entered
        
        Board_2(k) = Board_Temp(action); 
        Board_Temp(action) = 0; %make that box empty
        %Board_Temp
        Next_state = StateCalc(Board_Temp);  %Board is updated after the action is taken State is not updated here
        Next_state_array(m) = Next_state;
        m = m+1;
        [Reward,Winner,TerminalState] = RewardCalc(Board_2, color_3, color_2, color_1, k);
        B = k;
        k = k+1;
        
        [Q_Table, Q_Best_Next] = QBestValue(Q_Table,Board,Next_state,TerminalState);
        %Q_Table(state_2,:)
        Q_Table(state_2,action) = Q_Table(state_2,action) + alpha*(Reward + y*(Q_Best_Next) - Q_Table(state_2,action));
        %Q_Table(state_2,:)
        state_2 = Next_state;
        if TerminalState==1 && Winner == 1
            fprintf('\n Correct Assembly in %d state \n',i);
            Flag = 1;
            Board_2
        elseif TerminalState==1 && Winner == 2
            fprintf('\n Wrong Assembly \n');
            Flag = 2; %Wrong Assembly
            k = 1;
            Board_2 = [0 0 0 0 0 0];
        end
    end
end
save('Baxter_Training_2.mat','Q_Table');
            
                
                
                
        
    
    
    

