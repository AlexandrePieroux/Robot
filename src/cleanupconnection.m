function cleanupconnection(api, vrep)
	% Cleanup the connexion

	fprintf('Closing connection %d.\n', vrep);
	api.simxStopSimulation(vrep, api.simx_opmode_oneshot_wait);
	api.simxFinish(vrep);
	api.delete();
	disp('Program ended');
end
