# Changelog

## [1.2] - 2025-01-29

Tested to support CoppeliaSim up to v4.9.

### Fixed

- Replaced `boost::asio::ip::address::from_string` with `boost::asio::ip::make_address` to support current version of boost.

## [1.1] - 2024-01-12

### Added

- Support for CoppeliaSim v4.6

### Changed

- The models were changed to support CoppeliaSim 4.6, which requires to load the plugin explicitly. 
- The scenes `playground_tof_{ep|s1}.ttt` were updated with the new robot models.

## [1.0] - 2023-05-13

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
