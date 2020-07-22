1. run initialize to load the reference track. The sequence is s, X, Y, Vx, kap, psi.  
   The init states are [X,Y,psi,Vx,Vy,dpsi]  
   
2. racing_car2_MPC.slx is the simulink, please set the sampling rate as 60Hz in configuration.  
3. controller_MPC2.m is the controller main function. The weights are in  
https://github.com/CeHao1/GTS_MPC/blob/2b0528c2f0e782bbf2a10323f3770035a6de73da/Matlab_simulation/Control/controller_MPC2.m#L298  

  if you change the car, please modify parameters here 
  https://github.com/CeHao1/GTS_MPC/blob/8088638675252118f5f50b40adf528bd8a623cba/Matlab_simulation/Control/controller_MPC2.m#L319

4. When you finish the simulation, run function plotfun1,2 or 2_5 to see different resutls.
