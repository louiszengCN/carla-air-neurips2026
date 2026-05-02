# Building CarlaAir from source

This guide explains how to build the unified CARLA + AirSim runtime
described in paper §3 / Appendix B from source. The result is a single
`CarlaUE4-Linux-Shipping` binary that opens both the CARLA RPC port
(2000) and the AirSim RPC port (41451) on one process.

> **Note.** The from-source build is intended for users who need to
> customise the engine layer. Most reviewers can skip this guide and
> use a pre-built distribution: drop `CarlaAir.sh`, `auto_traffic.py`,
> `AirSimConfig/`, and `env_setup/` from this directory into a CarlaAir
> release tree and launch.

The build is the standard CARLA + UE4 build flow with AirSim added as
a UE4 plugin and three files replaced with the versions in
`source_modifications/`.

## 1. System requirements

- Ubuntu 18.04 / 20.04 / 22.04
- NVIDIA GPU, ≥ 8 GB VRAM
- ≥ 32 GB RAM
- ≥ 100 GB free disk space

## 2. System packages

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
                    clang-10 lld-10 \
                    libvulkan1 vulkan-utils mesa-vulkan-drivers \
                    python3-dev python3-pip
```

## 3. Python environment

```bash
conda create -n carlaAir python=3.8 -y
conda activate carlaAir
pip install carla==0.9.16 airsim numpy opencv-python pygame
```

## 4. Get the source trees

CarlaAir builds against three upstream codebases:

1. **Unreal Engine 4.26** — Epic's source release on GitHub (requires
   linking your GitHub account to your Epic Games account). Clone it
   somewhere and remember the path:

   ```bash
   git clone -b 4.26 https://github.com/EpicGames/UnrealEngine.git
   export UE4_ROOT=$PWD/UnrealEngine
   ```

2. **CARLA 0.9.16** — the `ue4-dev` branch:

   ```bash
   git clone --depth 1 -b 0.9.16 https://github.com/carla-simulator/carla.git
   export CARLA_ROOT=$PWD/carla
   ```

3. **AirSim 1.7.0** — the Colosseum community-maintained fork is
   recommended (the original Microsoft repo is no longer actively
   maintained). Drop its UE4 plugin into the CARLA tree:

   ```bash
   git clone --depth 1 https://github.com/CodexLabsLLC/Colosseum.git AirSim-source
   cp -r AirSim-source/Unreal/Plugins/AirSim \
         "$CARLA_ROOT/Unreal/CarlaUE4/Plugins/AirSim"
   ```

## 5. Apply the three modifications

Replace the corresponding files in the CARLA tree with the versions in
this repository's `source_modifications/`:

```bash
PATCH_DIR=<path-to-this-repo>/platform/source_modifications

cp "$PATCH_DIR/CarlaGameModeBase.h" \
   "$CARLA_ROOT/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.h"

cp "$PATCH_DIR/CarlaGameModeBase.cpp" \
   "$CARLA_ROOT/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.cpp"

cp "$PATCH_DIR/CarlaUE4.Build.cs" \
   "$CARLA_ROOT/Unreal/CarlaUE4/Source/CarlaUE4/CarlaUE4.Build.cs"
```

Why these three files: the `.h`/`.cpp` extend CARLA's `GameModeBase`
to instantiate the AirSim flight actor at world initialisation and
synchronise its lifecycle; the `.Build.cs` adds the AirSim plugin
module to the build dependency list. See
`source_modifications/README.md` for the file-level mapping back to
the paper's "CarlaUE4GameMode" naming.

## 6. Build

```bash
cd "$CARLA_ROOT"

# (a) Build the engine and base modules. May take several hours on first run.
make UnrealEngine

# (b) Build the AirSim plugin module.
make CarlaUE4Editor ARGS="-module=AirSim"

# (c) Build the CARLA module (now linked against the modified GameMode +
#     the AirSim plugin).
make CarlaUE4Editor ARGS="-module=Carla"
```

To run interactively in the editor for development:

```bash
make launch
```

To package a stand-alone Shipping build (the binary that
`CarlaAir.sh` expects):

```bash
./Util/BuildTools/Package.sh --config=Shipping --no-zip
```

The output lands under `Dist/CARLA_Shipping_*/LinuxNoEditor/`. Copy
(or symlink) its contents next to `CarlaAir.sh`:

```bash
cp -r Dist/CARLA_Shipping_*/LinuxNoEditor/* \
      <path-to-this-repo>/platform/
```

After this step, `<path-to-this-repo>/platform/` should contain a
`CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping` binary alongside
`CarlaAir.sh`. Launch with:

```bash
cd <path-to-this-repo>/platform
./CarlaAir.sh Town10HD --quality Epic
```

## 7. Verify the build

In a second terminal:

```bash
conda activate carlaAir
python -c "import carla;  c=carla.Client('localhost', 2000);  print(c.get_world().get_map().name)"
python -c "import airsim; c=airsim.MultirotorClient(port=41451); c.confirmConnection(); print('OK')"
```

Both calls should succeed against the single running process. The
`eval/` suite can then connect on top of the same ports.

## Troubleshooting

**Missing system headers (`features.h`, etc.)** — UE4's clang
toolchain sometimes does not pick up the system include paths. Use
the bundled `Build.sh` script instead of `make` for the failing
target.

**Editor crashes on launch** — confirm Vulkan is available
(`vulkan-utils` package, `vulkaninfo` runs cleanly). The first launch
also compiles a large shader cache; expect a multi-minute wait.

**Drone is missing in the packaged build** — make sure the AirSim
blueprint and asset directories are listed under
`DirectoriesToAlwaysCook` in
`Unreal/CarlaUE4/Config/DefaultGame.ini`, then re-run
`Package.sh`.
