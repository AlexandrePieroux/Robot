function startSimulation()
  % Launches the simulation

  disp('Connecting to VREP');
  % Setup the connexion with the VREP server
  api = remApi('remoteApi');
  api.simxFinish(-1);
  vrep = api.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

  if vrep < 0,
    disp('Failed connecting to remote API server. Exiting.');
    api.delete();
    return;
  end
  fprintf('Connection %d to remote API server open.\n', vrep);

  % Make sure we close the connexion whenever the script is interrupted.
  cleanupObj = onCleanup(@() cleanupconnection(api, vrep));

  disp('Starting the simultation');
  % Start the simulation (res not necessary in this mode)
  api.simxStartSimulation(vrep, api.simx_opmode_oneshot_wait);

  disp('Setting up the robot informations');
  robot = Robot(5, 6, 1.5, 2.5);  
  
  disp('Creating a brain');
  brain = Brain();
  
  disp('Brain initialization');
  brain.init(api, vrep, robot, 4, pi/90, 4);
  
  disp('Fly little bird, fly');
  brain.work();
end