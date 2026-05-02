# Source modifications

Paper §3 describes the runtime as preserving CARLA as the authoritative world
manager and composing the AirSim flight actor as an actor-level component
inside the same UE4 simulation. App. B summarises this as three modified
upstream files. The three files in this directory are the modifications:

| File here | Path inside the upstream tree | Role |
|---|---|---|
| `CarlaGameModeBase.h`   | `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.h`   | Adds the AirSim flight-actor declaration and composition pointer to CARLA's GameMode. The paper refers to this as the modified `CarlaUE4GameMode.h`. |
| `CarlaGameModeBase.cpp` | `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.cpp` | Instantiates the AirSim actor on `BeginPlay` and synchronises its lifecycle with CARLA's world. The paper refers to this as the modified `CarlaUE4GameMode.cpp`. |
| `CarlaUE4.Build.cs`     | `Unreal/CarlaUE4/Source/CarlaUE4/CarlaUE4.Build.cs`                     | Adds the AirSim UE4 plugin module to the build dependency list. |

To reproduce the runtime on top of vanilla CARLA + AirSim, replace those
three files in a CARLA 0.9.16 source tree (with AirSim 1.7.0 added as a UE4
plugin under `Plugins/AirSim/`) with the versions in this directory, then
rebuild.

The composition is one-way: CARLA owns the world lifecycle, traffic, weather,
sensor scheduling, and RPC services; AirSim is registered as a regular UE4
actor and exposes its own RPC server so existing AirSim clients keep working
unchanged. No other CARLA code is touched.
