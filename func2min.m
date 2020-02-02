function [IAE] = func2min(x)
% Runs the simulation with a given values of Kp, Kd, and Ki and returns the
% IAE (see p22 of 'Control PID.pdf'). In the Aerogen2019ControllerStep the
% time of the simulation and steps can be configured, at this moment the
% simulation is performed for 10 seconds from the trim state and: 
%   1.- a Step in the comanded speed is generated at t=0 from 50 to 45 rad/s
%   2.- a step in wind speed is generated at t=5 from 10 to 11 m/s
% IAE = ?|e|dt
% ISE = ?e^2dt
% IATE = ?t|e|dt


    Kp = x(1);
    Kd = x(2);
    Ki = x(3);
    
    in = Simulink.SimulationInput('Aerogen2019ControllerStep');
    in = in.setBlockParameter('Aerogen2019ControllerStep/PID_Kd','Value',string(Kd));
    in = in.setBlockParameter('Aerogen2019ControllerStep/PID_Kp','Value',string(Kp));
    in = in.setBlockParameter('Aerogen2019ControllerStep/PID_Ki','Value',string(Ki));
    %in = in.setBlockParameter('Aerogen2019ControllerStep/Constant1','value',string(50));
    out = sim(in);
    %ISE = sum(out.yout.^2);
    %IAE = sum(abs(out.yout));
    IAE = out.yout(end);
    
end
