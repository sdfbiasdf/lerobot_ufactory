# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LeRobot is a Hugging Face robotics framework for imitation learning and reinforcement learning in PyTorch. This is a **fork** (`sdfbiasdf/lerobot_ufactory`) of the upstream xArm-Developer/lerobot repo (itself forked from huggingface/lerobot), with UFFactory xArm-specific customizations for Gello teleoperation and pi0.5 evaluation configs.

- Python 3.10+, PyTorch 2.2+
- Version: 0.4.3
- Package source: `src/lerobot/`

## Build & Install

```bash
pip install -e .                    # Editable install (core only)
pip install -e ".[aloha,pusht]"     # With simulation extras
pip install -e ".[all]"             # All optional dependencies
pip install -e ".[test]"            # Test dependencies
pip install -e ".[dev]"             # Dev + pre-commit dependencies
```

## Common Commands

```bash
# Lint & format
pre-commit run --all-files          # Full quality checks (ruff, mypy, bandit, typos)
ruff check src/lerobot              # Lint only
ruff format src/lerobot             # Format only

# Tests
pytest tests/ -v                    # Run all tests
pytest tests/policies/ -v           # Test specific module
pytest tests/test_available.py -v   # Single test file
DEVICE=cuda pytest tests/ -v       # GPU tests

# End-to-end training/eval tests (via Makefile)
make test-end-to-end                # All 4 policies (ACT, Diffusion, TDMPC, SmolVLA)
make test-act-ete-train DEVICE=cpu  # Single policy E2E test

# CLI tools
lerobot-train --policy.type=act ... # Train a policy
lerobot-eval --policy.path=... ...  # Evaluate a policy
lerobot-record                      # Record dataset from robot
lerobot-teleoperate                 # Teleoperation control
lerobot-calibrate                   # Robot calibration
lerobot-find-cameras                # Discover cameras
lerobot-find-port                   # Discover serial ports
lerobot-dataset-viz                 # Visualize dataset with rerun.io
```

## Code Style

- Line length: 110 characters
- Formatter/linter: ruff (rules: E, W, F, I, B, C4, T20, N, UP, SIM)
- Quote style: double
- Docstring convention: Google
- Import sorting: isort via ruff, `lerobot` as known first-party
- Type checking: gradual mypy adoption (strict for configs, envs, models, cameras, transport)

## Architecture

### Factory Pattern
Core components use factory functions for instantiation:
- `policies/factory.py` → `make_policy()`, `get_policy_class()`
- `datasets/factory.py` → `make_dataset()`
- `envs/factory.py` → `make_env()`
- `processor/factory.py` → processor creation
- `optim/factory.py` → optimizer/scheduler creation

### Configuration System
Uses `draccus` (dataclass-based CLI config parsing). Config dataclasses live in `src/lerobot/configs/`:
- `TrainPipelineConfig` — training pipeline
- `EvalConfig` — evaluation
- `PreTrainedConfig` — policy configs
- `EnvConfig` / `RobotConfig` — env and robot configs

### Policy Architecture
All policies extend `PreTrainedPolicy` (nn.Module + HubMixin + ABC) in `policies/pretrained.py`. Each policy has its own subdirectory with `configuration_*.py`, `modeling_*.py`, and optionally `processor_*.py`. Available: act, diffusion, tdmpc, vqbet, smolvla, pi0, pi05, groot, xvla, rtc, sac.

### Hardware Abstraction
- **Robots** (`robots/`): ABC `Robot` base class. Implementations: koch, aloha, so100/101, ufactory_robot, hope_jr, etc.
- **Teleoperators** (`teleoperators/`): ABC `Teleoperator` base class. Implementations: gello_xarm, keyboard, gamepad, space_mouse, phone, etc.
- **Cameras** (`cameras/`): opencv, intelrealsense drivers
- **Motors** (`motors/`): dynamixel, feetech drivers

### Dataset System
`LeRobotDataset` (PyTorch Dataset) in `datasets/lerobot_dataset.py`:
- Parquet for structured data, MP4 for video
- Supports temporal queries via `delta_timestamps`
- Hugging Face Hub integration for upload/download
- Online buffer for real-time recording

### UFFactory-Specific Code
Located in `src/lerobot/ufactory_usage/`: configs and scripts for xArm recording, evaluation, and teleoperation testing. Uses YAML configs in `ufactory_usage/config/`.

## Adding New Components

When adding a **new policy**: update `available_policies` in `src/lerobot/__init__.py`, set the `name` class attribute, update `tests/test_available.py`.

When adding a **new environment**: update `available_tasks_per_env` and `available_datasets_per_env` in `src/lerobot/__init__.py`.

## Git Remotes

- `origin`: sdfbiasdf/lerobot_ufactory (this fork)
- `upstream`: xArm-Developer/lerobot
- Original source: huggingface/lerobot
