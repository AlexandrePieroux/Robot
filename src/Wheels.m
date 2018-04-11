classdef Wheels < handle
    
  properties
    flHandle;
    rlHandle;
    frHandle;
    rrHandle;
    maxlv;
    minav;
    maxav;
  end
  
  methods
    function obj = Wheels(maxlv, minav, maxav)
        obj.maxlv = maxlv;
        obj.minav = minav;
        obj.maxav = maxav;
    end
    
    function init(obj, api, vrep)
        [res, obj.flHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_fl', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.rlHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_rl', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.rrHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_rr', api.simx_opmode_oneshot_wait); vrchk(api, res);
        [res, obj.frHandle] = api.simxGetObjectHandle(vrep, 'rollingJoint_fr', api.simx_opmode_oneshot_wait); vrchk(api, res);
    end
  end
    
end

