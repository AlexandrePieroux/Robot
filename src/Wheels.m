classdef Wheels < handle
    
  properties
    wheelRadius;
      
    flHandle;
    rlHandle;
    frHandle;
    rrHandle;
    
    swFlHandle;
    swRlHandle;
    swFrHandle;
    swRrHandle;
  end
  
  methods
    function obj = Wheels()
    end
    
    function init(obj, api, vrep)   
        % Get the wheels handles
        [res, obj.flHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_fl', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.rlHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_rl', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.rrHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_rr', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.frHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_fr', api.simx_opmode_oneshot_wait); vrchk(api, res);
        
        [res, obj.swFlHandle] = api.simxGetObjectHandle(vrep, 'swedishWheel_fl', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.swRlHandle] = api.simxGetObjectHandle(vrep, 'swedishWheel_rl', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.swFrHandle] = api.simxGetObjectHandle(vrep, 'swedishWheel_fr', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.swRrHandle] = api.simxGetObjectHandle(vrep, 'swedishWheel_rr', api.simx_opmode_oneshot_wait); vrchk(api, res);

        % Get the radius of the wheel
        [res, obj.wheelRadius] = api.simxGetObjectFloatParameter(vrep, obj.swRrHandle, 20, api.simx_opmode_oneshot_wait); vrchk(api, res);        
    end
  end
    
end

