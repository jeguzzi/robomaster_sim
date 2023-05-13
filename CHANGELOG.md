# Changelog

## [Unreleased]

### Added

- Support for CoppeliaSim v4.5. The plugin supports now versions from 4.0 to 4.5.

### Changed

- The model was changed to support CoppeliaSim 4.5. In particular the initializer
  `simRobomaster.create` and `simRobomaster.create_{ep|s1}` now take the model handle as first argument,
  while previously they took the model index. When using older scenes with Robomasters, 
  users need to update they robots to use the new initializer, 
  even if they are using a previous version of CoppeliaSim.
- The two scenes `playground_tof_{ep|s1}.ttt` were updated with the new robot models.

### Known issues

- The initial release Coppelia v4.5 has [a bug](https://forum.coppeliarobotics.com/viewtopic.php?p=38863) 
  in the lua script of `simIK`. If you experience errors
  using the Robomaster gripper, add the following lines at the begin 
  of `simIK.debugGroupIfNeeded` in `simIK.lua`
  ```lua
  function simIK.debugGroupIfNeeded(ikEnv,ikGroup,debugFlags)
    if not _S.ikEnvs then
        _S.ikEnvs={}
    end
    ...
  ```	

## [0.1] - 2023-04-05

Initial release
