function stateChanges = NonLinearClosedLoop(t,state,r,nu,mu,m,I_B,g,k,k1_Lat,k2_Lat,k1_Long,k2_Long)
    %{
        Inputs: 
                t, time in seconds
                state, current state values
                f_motor, 4x1 vector containing scalar force output of each motor
                r, radius of motors from center of mass, meters
                k_m, control moment coefficient
                nu, aerodynamic force coefficient
                mu, aerodynamic moment coefficient
                m, drone mass
                I_B, drone moment of inertial tensor, body comp
                g, acceleration due to gravity
                perturbance, small number used to express a flight
                    distruption, nominally 0
        Outputs:
                stateChanges, how each state value will change in the next
                instant of time
        Methodology:
                This function for use exclusively inside of ODE45; it takes
                advantage of the Quadrotor Equations of Motion to determine
                how a simulated quadrotor aircraft with behave given
                its current state and the current forces and moments on it
                (control, aerodynamic)
    %}
%%% Breaking down the state vector into easier-to-use vectors
    pos = state(1:3); % m; position
    EA = state(4:6); %  radians for crying out loud; Euler angle
    vel = state(7:9) ; % m/s, velocity
    omega = state(10:12) ; % rad/s
    
%%% Calculating Aerodynamic Forces
    
    airspeed = norm(vel);
    if airspeed ~= 0 % To avoid dividing by zero
        drag_magnitude = nu*airspeed^2;
        drag_direction = - vel / airspeed;
        f_aero = drag_magnitude * drag_direction; % aerodynamic forces from drag, (N)
    else
        f_aero = [0;0;0];
    end
%     
%%% Calculating f_motor from control force
    Zc = -m*g; % weight of the quadrotor, N
%     

%%% Calculating Control and Aerodynamic Moments
    
L_c = -k1_Lat*omega(1) - k2_Lat*EA(1);
M_c = -k1_Long*omega(2) - k2_Long*EA(2);
N_c = -k*omega(3);
   
Gc = [L_c; M_c;N_c];

    G = -mu * sqrt(omega(1)^2 + omega(2)^2 + omega(3)^2) * omega; % Aerodynamic moments based on drag, rotational velocities
   
%%% Forming stateChanges components, using Quadrotor Equations of Motion
    % Change in position EOM
    xyz_dot = [ (cos(EA(2))*cos(EA(3))), (sin(EA(1))*sin(EA(2))*cos(EA(3)) - (cos(EA(1))* sin(EA(3)))), (cos(EA(1))*sin(EA(2))*cos(EA(3)) + (sin(EA(1))*sin(EA(3))));... 
                (cos(EA(2))*sin(EA(3))), (sin(EA(1))*sin(EA(2))*sin(EA(3)) + (cos(EA(1))* cos(EA(3)))), (cos(EA(1))*sin(EA(2))*sin(EA(3)) - (sin(EA(1))*cos(EA(3))));...
                (-sin(EA(2)))          , (sin(EA(1))*cos(EA(2)))                                      , (cos(EA(1))*cos(EA(2)))                                    ] * vel;
    % Change in angle EOM
    EA_dot =  [ 1, (sin(EA(1))*tan(EA(2))), (cos(EA(1))*tan(EA(2))) ;...
                0,      (cos(EA(1)))       ,      (-sin(EA(1)))      ;...
                0, (sin(EA(1))*sec(EA(2))), (cos(EA(1))*sec(EA(2)))] * omega;
    % Change in velocity EOM       
    vel_dot = [ (omega(3) * vel(2) - omega(2) * vel(3));...
                (omega(1) * vel(3) - omega(3) * vel(1));...
                (omega(2) * vel(1) - omega(1) * vel(2))]...
              + g*...
              [-sin(EA(2));...
              (cos(EA(2))*sin(EA(1)));...
              (cos(EA(2))*cos(EA(1)))]...
              + (1/m)*...
              [f_aero(1);...
               f_aero(2);...
               f_aero(3)]...
              + (1/m)*...
              [0;...
               0;...
               Zc];
    % Change in angular velcoity EOM     
    omega_dot = [ (((I_B(2,2) - I_B(3,3)) / I_B(1,1)) * omega(2)*omega(3));...
                  (((I_B(3,3) - I_B(1,1)) / I_B(2,2)) * omega(1)*omega(3));...
                  (((I_B(1,1) - I_B(2,2)) / I_B(3,3)) * omega(1)*omega(2))]...
                + ...
                [ (G(1)/I_B(1,1));...
                  (G(2)/I_B(2,2));...
                  (G(3)/I_B(3,3))]...
                + ...
                [ (Gc(1)/I_B(1,1));...
                  (Gc(2)/I_B(2,2));...
                  (Gc(3)/I_B(3,3))]; 
              
    % The overall change in the state vector          
    stateChanges = [ xyz_dot; EA_dot; vel_dot; omega_dot];
              
end
