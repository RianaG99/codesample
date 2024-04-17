%Leader Agent, Agent 0
L0 = [-25,0,pi];

% Agent Initial conditions, Agents 1 through N
A10 = [20,5,pi]; %2WMR
A20 = [10,17.32,5,0,0,0,0,0,0,0,0,0]; %Quad
A30 = [-10,17.32,-pi/3];%2WMR
A40 = [-20,4,5,0,0,0,0,0,0,0,0,0]; %Quad
A50 = [-10,-17.32,pi/3];%2WMR
A60 = [10,-17.32,5,0,0,0,0,0,0,0,0,0]; %Quad

%Based on topology, Laplacian matrix of the graph can be find out,whose eigenvalues will be (2, 1, 1, 1, 1, 1, 0)
G = digraph([1,2,3],[2,3,1]);  %digraph directs
%compute laplacian of directed graph to match results of paper
A = G.adjacency;
Din = diag(sum(A,1)); % in degree matrix
Dout = diag(sum(A,2)); % in degree matrix
Lin = Din - A; % laplacian matrix
L = full(Lin);
L = transpose(L);
e = eig(L)'; 
%% Drones
%L = [1 0 1; 1 1 0; 0 1 1];

L = [0 0 0 0;1 1 0 1;0 1 1 0;0 0 1 1];
L0 = [-25,0,pi];
controls = [.1,0];
path = [L0];
tspan = [0:.2: 5]; %this allows it not to overshoot (could be smaller maybe if add dynamic controls)

xd_s = 5;
yd_s = 1;
path20 = [];
path40 = [];
path60 = [];
time_go = 0;
while time_go < 100
    
    
%Leader state after time tk
[t,leader_state] = ode45(@(t,state) kinocar(state,controls),tspan,L0);
L0 = leader_state(end,:);
path = [path;leader_state];

if L0(1) < -48
    controls = [0 0];
end
A20 = A20';
A40 = A40';
A60 = A60';
    
xd = (L*[L0(1);A20(1);A40(1);A60(1)])./sum(L,2);
yd = (L*[L0(2);A20(2);A40(2);A60(2)])./sum(L,2);
xHistoryA20 = nlmpccontrol(A20,[xd(2),yd(2)]);
xHistoryA40 = nlmpccontrol(A40,[xd(3),yd(3)]);
xHistoryA60 = nlmpccontrol(A60,[xd(4),yd(4)]);
A20 = xHistoryA20(end,:);
A40 = xHistoryA40(end,:);
A60 = xHistoryA60(end,:);
path20 = [path20;xHistoryA20];
path40 = [path40;xHistoryA40];
path60 = [path60;xHistoryA60];
time_go = time_go + 5;
disp(time_go)

end
%%
figure(1)
hold on
plot3(path(:,1),path(:,2),path(:,3))
plot3(path20(:,1),path20(:,2),path20(:,3))
plot3(path40(:,1),path40(:,2),path40(:,3))
plot3(path60(:,1),path60(:,2),path60(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
title('Convergence of Quadrotor States Under Proposed Consensus Protocol')
hold off
%% Car Model
%Based on topology, Laplacian matrix of the graph can be find out,whose eigenvalues will be (2, 1, 1, 1, 1, 1, 0)
G = digraph([1,2,3,4],[2,3,4,2]);  %digraph directs
%compute laplacian of directed graph to match results of paper
A = G.adjacency;
Din = diag(sum(A,1)); % in degree matrix
Dout = diag(sum(A,2)); % in degree matrix
Lin = Din - A; % laplacian matrix
L = full(Lin);
L = transpose(L);
e = eig(L)'; 

L = [ 1 0 1; 1 1 0; 0 1 1];
L0 = [-25,0,pi];
controls = [.07,0];
path = [L0];
tspan = [0:.2: 5]; %this allows it not to overshoot (could be smaller maybe if add dynamic controls)

% Agent Initial conditions, Agents 1 through N
A10 = [20,5,pi]; %2WMR
A30 = [-10,17.32,-pi/3];%2WMR
A50 = [-10,-17.32,pi/3];%2WMR

dynstate1 = [0 0 0 0];
dynstate3 = [0 0 0 0];
dynstate5 = [0 0 0 0];

path10 = [A10];
path30 = [A30];
path50 = [A50];
time_go = 0;

while time_go < 200
    
%Leader state after time tk
%[t,leader_state] = ode45(@(t,state) kinocar(state,controls),tspan,L0);
%L0 = leader_state(end,:);
%path = [path;leader_state];

%if L0(1) < -48
 %   controls = [0 0];
%end
    

xd = (L*[A10(1);A30(1);A50(1)])./(sum(L,2));
yd = (L*[A10(2);A30(2);A50(2)])./(sum(L,2));


[car_state1,dynamic_state1] = TwowmrMove(A10,[xd(1) yd(1) 0],dynstate1,1.1,10.051,.08,.1,.51,.8);
[car_state3,dynamic_state3] = TwowmrMove(A30,[xd(2) yd(2) 0],dynstate3,1.1,10.051,0.08,.1,.51,.8);
[car_state5,dynamic_state5] = TwowmrMove(A50,[xd(3) yd(3) 0],dynstate5,1.1,10.051,0.08,.1,.51,.8);

A10 = car_state1(end,:);
dynstate1 = dynamic_state1;

A30 = car_state3(end,:);
dynstate3 = dynamic_state3;

A50 = car_state5(end,:);
dynstate5 = dynamic_state5;

path10 = [path10;A10];
path30 = [path30;A30];
path50 = [path50;A50];

 
time_go=time_go+5;
disp(time_go)

end

ground = zeros(1,length(path10(:,1)));
gt = zeros(1,length(path(:,1)));
figure(1)
hold on
plot3(path(:,1),path(:,2),gt,'--')
plot3(path10(:,1),path10(:,2),ground,'b')
plot3(path30(:,1),path30(:,2),ground)
plot3(path50(:,1),path50(:,2),ground)
xlabel('x')
ylabel('y')
legend('L','A1','A2','A3')
title('Convergence of Quadrotor States Under Proposed Consensus Protocol')
hold off

%%

L0 = [-25,0,pi];
controls = [.07,0];
path = [L0];
tspan = [0:.2: 5]; %this allows it not to overshoot (could be smaller maybe if add dynamic controls)
tm = 0;
while tm < 75
[t,leader_state] = ode45(@(t,state) kinocar(state,controls),tspan,L0);
L0 = leader_state(end,:);
path = [path;leader_state];
tm = tm+1;
end

plot(path(:,1),round(path(:,2)))


%% Full Model 

L = [0 0 0 0 0 0 0; 0 1 1 0 0 0 1; 0 0 1 1 0 0 0; 0 0 0 1 1 0 0; 1 0 0 0 1 0 0 ; 0 0 0 0 1 1 0; 0 0 0 0 0 1 1];
tspan = [0:.2: 5]; %this allows it not to overshoot (could be smaller maybe if add dynamic controls)

%Leader Agent, Agent 0
L0 = [-25,0,pi];
controls = [.07,0];
path = [L0];
tspan = [0:.2: 5]; %this allows it not to overshoot (could be smaller maybe if add dynamic controls)

% Agent Initial conditions, Agents 1 through N
A10 = [20,5,pi]; %2WMR
A20 = [10,17.32,5,0,0,0,0,0,0,0,0,0]; %Quad
A30 = [-10,17.32,-pi/3];%2WMR
A40 = [-20,4,5,0,0,0,0,0,0,0,0,0]; %Quad
A50 = [-10,-17.32,pi/3];%2WMR
A60 = [10,-17.32,5,0,0,0,0,0,0,0,0,0]; %Quad

%store car states
path10 = [A10];
path30 = [A30];
path50 = [A50];
dynstate1 = [0 0 0 0];
dynstate3 = [0 0 0 0];
dynstate5 = [0 0 0 0];


%store drone states
path20 = [A20];
path40 = [A40];
path60 = [A60];
time_go = 0;

while time_go < 5

%Leader state after time tk
[t,leader_state] = ode45(@(t,state) kinocar(state,controls),tspan,L0);
L0 = leader_state(end,:);
path = [path;leader_state];

if L0(1) > 48
    controls = [0 0];
end

%get the desired x and y positions
xd = (L*[L0(1);A10(1);A20(1);A30(1);A40(1);A50(1);A60(1)])./sum(L,2);
yd = (L*[L0(2);A10(2);A20(2);A30(2);A40(2);A50(2);A60(2)])./sum(L,2);

%Drone Path storing
A20 = A20';
A40 = A40';
A60 = A60';
    
xHistoryA20 = nlmpccontrol(A20,[xd(3),yd(3)]);
xHistoryA40 = nlmpccontrol(A40,[xd(5),yd(5)]);
xHistoryA60 = nlmpccontrol(A60,[xd(7),yd(7)]);

A20 = xHistoryA20(end,:);
A40 = xHistoryA40(end,:);
A60 = xHistoryA60(end,:);
path20 = [path20;xHistoryA20];
path40 = [path40;xHistoryA40];
path60 = [path60;xHistoryA60];


% Car path storing
[car_state1,dynamic_state1] = TwowmrMove(A10,[xd(2) yd(2) 0],dynstate1,1.1,0.051,.08,.1,.51,.8);
[car_state3,dynamic_state3] = TwowmrMove(A30,[xd(4) yd(4) 0],dynstate3,1.1,0.051,0.08,.1,.51,.8);
[car_state5,dynamic_state5] = TwowmrMove(A50,[xd(6) yd(6) 0],dynstate5,1.1,0.051,0.08,.1,.51,.8);

A10 = car_state1(end,:);
dynstate1 = dynamic_state1;

A30 = car_state3(end,:);
dynstate3 = dynamic_state3;

A50 = car_state5(end,:);
dynstate5 = dynamic_state5;

path10 = [path10;A10];
path30 = [path30;A30];
path50 = [path50;A50];


time_go=time_go+5;

end

%% Plot results
ground = zeros(1,length(path10));

figure(1)
hold on
plot3(path(:,1),path(:,2),ground)
plot3(path10(:,1),path10(:,2),ground)
plot3(path30(:,1),path30(:,2),ground)
plot3(path50(:,1),path50(:,2),ground)
plot3(path20(:,1),path20(:,2),path20(:,3))
plot3(path40(:,1),path40(:,2),path40(:,3))
plot3(path60(:,1),path60(:,2),path60(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
title('Convergence of Quadrotor States Under Proposed Consensus Protocol')
hold off
