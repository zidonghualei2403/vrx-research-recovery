# VRX research recovery snapshot

This repository is a selected copy from the UGREEN backup of the old Ubuntu home directory. Original files on the external drive were not changed.

## Contents

- `src/vrx_csd_shadow/`: main ROS/Gazebo package and model JSON files.
- `c119/vrx_csd_shadow/`: source snapshot for the C119 medium-sea candidate.
- `project_scripts_and_notes/`: top-level run, analysis, and documentation files.
- `data_summary/`: small aggregate CSV files. Full raw runs remain in the offline backup.
- `environment/`: package versions observed in the copied `vrx_rl` Conda environment. These lists are an inventory, not a tested install recipe.

The upstream VRX source checkout in the backup is at commit `7609d1bd90ce7edb29d040a082f949e8b089c864`. The copied checkout has no content changes after disregarding executable-bit differences caused by the exFAT backup drive.

The old Ubuntu home backup does not include `/opt/ros/jazzy`. That directory has since been seen on the old SSD's Linux partition, but it is not needed in this repository. Reinstall ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04, then rebuild the ROS workspace and Python environment.

This repository excludes raw experiment logs, build/install outputs, Conda binaries, Codex conversations, credentials, and personal home-directory files. Verify the environment and experiments on a new Ubuntu installation before treating results as reproduced.
