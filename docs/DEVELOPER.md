# Duet3Expansion Developer Setup Guide

This guide explains how to set up a development environment for Duet3Expansion from scratch. No prior knowledge of VS Code or Docker is assumed.

---

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Install Required Software](#install-required-software)
4. [Clone the Repository](#clone-the-repository)
5. [Open the Project in VS Code](#open-the-project-in-vs-code)
6. [Open in Dev Container](#open-in-dev-container)
7. [Initialize Submodules](#initialize-submodules)
8. [Build the Firmware](#build-the-firmware)
9. [Build Targets](#build-targets)
10. [Debug vs Release Builds](#debug-vs-release-builds)
11. [Build Output and Packaging](#build-output-and-packaging)
12. [Using Git inside the Dev Container](#using-git-inside-the-dev-container)
13. [Git Tools: Git Graph and GitLens](#git-tools-git-graph-and-gitlens)
14. [Troubleshooting](#troubleshooting)

---

## Overview

Duet3Expansion is firmware for Duet 3 expansion and tool boards based on ARM Cortex-M processors (SAMC21 and SAME51 families).

The project uses a **dev container**: a self-contained Linux build environment defined in code. When you open this repository in VS Code, VS Code can build and start the container automatically. Inside the container, required tools such as the ARM GNU toolchain and `make` are already configured.

---

## Prerequisites

You need three things installed on your computer before starting:

- **Git**: for cloning the repository
- **Docker Desktop** (or Docker Engine on Linux): for the containerized build environment
- **Visual Studio Code (VS Code)**: the editor

### What is Docker?

Docker runs isolated environments called **containers**. Each container has its own OS packages, compilers, and tools. This repository's dev container provides the toolchain and build dependencies needed for Duet3Expansion.

### What is VS Code?

VS Code is a free code editor. The **Dev Containers** extension can automatically build and start the Docker container, then connect your editor to it so you can write and build code inside that environment.

---

## Install Required Software

### 1. Git

- **Windows / macOS**: download from https://git-scm.com/downloads
- **Ubuntu/Debian Linux**: `sudo apt install git`

Verify installation:

```sh
git --version
```

### 2. Docker

Install Docker Desktop from https://www.docker.com/products/docker-desktop/ (or Docker Engine on Linux).

Verify Docker is running:

```sh
docker --version
```

### 3. Visual Studio Code

Install VS Code from https://code.visualstudio.com/

Install the **Dev Containers** extension (by Microsoft):

1. Open VS Code.
2. Open the Extensions view.
3. Search for `Dev Containers`.
4. Click **Install**.

---

## Clone the Repository

Open a terminal and run:

```sh
git clone https://github.com/Duet3D/Duet3Expansion.git
cd Duet3Expansion
```

> [!NOTE]
> You can initialize submodules now with `--recurse-submodules`, or do it later with `make init-submodules`.

> [!NOTE]
> Check out the branch you intend to build before initializing submodules, for example:
>
> ```sh
> git checkout 3.7-docker
> ```

---

## Open the Project in VS Code

1. Open VS Code.
2. Choose **File -> Open Folder...** and select the `Duet3Expansion` folder.

Or from a terminal in the repository:

```sh
code .
```

---

## Open in Dev Container

When VS Code opens the folder, it should detect `.devcontainer/` and offer:

> **"Folder contains a Dev Container configuration file. Reopen in Container?"**

Click **Reopen in Container**.

If you miss it:

1. Open the Command Palette (`F1` / `Ctrl+Shift+P` / `Cmd+Shift+P`).
2. Run `Dev Containers: Reopen in Container`.

The first build may take several minutes while the image and toolchain are prepared.

> [!NOTE]
> All terminals opened in VS Code after this run inside the container.

---

## Initialize Submodules

This repository depends on Git submodules under `libraries/` (for example `CANlib`, `CoreN2G`, `FreeRTOS`, and `RRFLibraries`). You must initialize these before building.

Run:

```sh
make init-submodules
```

This checks out the pinned submodule commits used by the current branch.

### Automatically update submodules on branch switch

You can configure Git to recurse into submodules automatically when switching branches or pulling:

```sh
git config --global submodule.recurse true
```

With this enabled, branch changes also move submodules to the branch-pinned commits.

---

## Build the Firmware

With submodules initialized, build a board target:

```sh
make EXP3HC -j
```

You can also use VS Code tasks:

1. Press `Ctrl+Shift+B` (or `Cmd+Shift+B` on macOS).
2. Select **Build Selected Target**.
3. Choose a target and build type when prompted.

---

## Build Targets

Current top-level `make` targets are:

| `make` target | Hardware family |
|---|---|
| `EXP1HCL` | Duet 3 Expansion 1HCL (SAME51) |
| `EXP1XD` | Duet 3 Expansion 1XD (SAMC21) |
| `EXP3HC` | Duet 3 Expansion 3HC (SAME51) |
| `F3PTB` | F3PTB (SAME51) |
| `M23CL` | M23CL (SAME51) |
| `SAMMYC21` | Sammy C21 (SAMC21) |
| `SZP` | SZP (SAMC21) |
| `TOOL1LC` | Duet 3 Tool 1LC (SAMC21) |
| `TOOL1RR` | Tool 1RR (SAME51) |
| `TOOLINDX` | Tool Index (SAME51) |
| `all` | Build all configurations |

### Other useful targets

| `make` target | Description |
|---|---|
| `init-submodules` | Initialize/update pinned library submodules |
| `clean` | Remove build outputs for all targets |
| `clean-<config>` | Remove build outputs for one target, e.g. `clean-EXP3HC` |
| `clean-all` | Remove build outputs and clean library builds |
| `test-toolchain` | Verify toolchain is accessible |
| `help` | Print available targets and options |

---

## Debug vs Release Builds

Release is the default. To build with debug symbols and lower optimization:

```sh
make DEBUG=1 EXP3HC -j
```

To show full compiler commands:

```sh
make V=1 EXP3HC
```

---

## Build Output and Packaging

Each target writes artifacts into a directory named after the target, for example `EXP3HC/`.

Typical outputs are:

- `.elf`: linked image with symbols
- `.bin`: firmware binary
- `.map`: linker map

Example output names for `EXP3HC`:

- `EXP3HC/Duet3Firmware_EXP3HC.elf`
- `EXP3HC/Duet3Firmware_EXP3HC.bin`
- `EXP3HC/Duet3Firmware_EXP3HC.map`

The build also attempts to run `CrcAppender` on the `.bin` file when available.

---

## Using Git inside the Dev Container

Git is available in the container and works normally in the repository.

Common commands:

```sh
git status
git log --oneline -10
git diff
git add .
git commit -m "message"
git checkout -b feature/xyz
git push
git pull
```

If needed, configure identity per-repository:

```sh
git config user.name "Your Name"
git config user.email "your@email.com"
```

Use `git submodule status` after branch switches to confirm submodule revisions are correct.

---

## Git Tools: Git Graph and GitLens

The following VS Code extensions are useful for history and branch review:

- **Git Graph** (mhutchie)
- **GitLens - Git supercharged** (GitKraken)

Typical workflow:

1. Switch branch in Source Control / Git Graph.
2. Confirm submodule revisions.
3. Run `make init-submodules` if needed.
4. Compare branch differences before building.

---

## Troubleshooting

### Docker/container startup fails

Ensure Docker is running and the **Dev Containers** extension is installed.

### Toolchain not found

Run:

```sh
make test-toolchain
```

If this fails outside the dev container, either install the ARM GNU toolchain or set `CROSS_COMPILE` explicitly:

```sh
make CROSS_COMPILE=/path/to/arm-none-eabi- EXP3HC
```

### Submodule directories are empty or out of sync

Run:

```sh
make init-submodules
```

If needed:

```sh
git submodule update --init --recursive
```

### First container build is slow

The first dev container build downloads and installs dependencies. Later starts reuse the cached image and are much faster.
