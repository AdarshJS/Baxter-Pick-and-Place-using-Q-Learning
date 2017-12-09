%Experiment file
 A = [1 2 3; 4 5 6; 7 8 9];
 %B = A(3,:)
 %k = find(A == 0)
 
 example = matfile('States.mat');
state = example.state;

k = find(state==2556)