local simRobomaster = {}

if loadPlugin then
    simRobomaster = loadPlugin 'simRobomaster';
    (require 'simRobomaster-typecheck')(simRobomaster)
end

function simRobomaster.wait_for_completed(robot_handle, action_handle)
    local savedLockLevel=sim.setThreadAutomaticSwitch(false) -- forbid automatic switches
    while true do
        local s = simRobomaster.get_action_state(robot_handle, action_handle)
        if(s ~= "running" and s ~= "started") then
            break
        end
        sim.switchThread()
    end
    sim.setThreadAutomaticSwitch(savedLockLevel)
end

function simRobomaster.wait_for_gripper(robot_handle, target_state)
    local savedLockLevel=sim.setThreadAutomaticSwitch(false) -- forbid automatic switches
    while true do
        local s = simRobomaster.get_gripper(robot_handle)
        if(s == target_state) then
            break
        end
        sim.switchThread()
    end
    sim.setThreadAutomaticSwitch(savedLockLevel)
end

function simRobomaster.open_gripper(...)
  local robot_handle, wait=checkargs({{type='int'}, {type='bool', default=true}},...)
  simRobomaster.set_gripper_target(robot_handle, "open")
  if wait then
    simRobomaster.wait_for_gripper(robot_handle, "open")
  end
end

function simRobomaster.close_gripper(...)
  local robot_handle, wait=checkargs({{type='int'}, {type='bool', default=true}},...)
  simRobomaster.set_gripper_target(robot_handle, "close")
  if wait then
    simRobomaster.wait_for_gripper(robot_handle, "close")
  end
end

function simRobomaster.init()
    -- can only be executed once sim.* functions were initialized
    sim.registerScriptFunction('simRobomaster.wait_for_completed@simRobomaster','simRobomaster.wait_for_completed(number action_handle)')
    sim.registerScriptFunction('simRobomaster.open_gripper@simRobomaster','simRobomaster.open_gripper(number robot_handle, bool wait=false)')
    sim.registerScriptFunction('simRobomaster.close_gripper@simRobomaster','simRobomaster.close_gripper(number robot_handle, bool wait=false)')
    simRobomaster.init=nil
end

if not __initFunctions then
    __initFunctions={}
end
__initFunctions[#__initFunctions+1]=simRobomaster.init

return simRobomaster
