function ReturnStates()
state = zeros(60,1);
color_3_array = zeros(360,1);
color_2_array = zeros(360,1);
color_1_array = zeros(360,1);
i = 1;
while (i<=60)
    [Board, color_3, color_2, color_1 ] = CreateRandomBoard_Simple();
    state_temp = StateCalc(Board)
    if (ismember(state_temp,state)==0)
        state(i) = state_temp;
        color_3_array(i) = color_3;
        color_2_array(i) = color_2;
        color_1_array(i) = color_1;
        i = i+1;
    end
    Size = size(find(state~=0))
end
state 
save('States.mat','state');
save('Color_3.mat','color_3_array');
save('Color_2.mat','color_2_array');
save('Color_1.mat','color_1_array');