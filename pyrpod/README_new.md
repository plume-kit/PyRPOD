# Project Overview

This repository contains a set of Python modules for mission planning, vehicle simulations, plume modeling, and utility functions for aerospace research and development. The modules are organized into directories based on functionality and purpose.

## Directory Structure

### Root Files
- **README.md**: Provides an overview of the project structure and usage instructions.
- **config_test.py**: Script for testing configuration setups.
- **pyrpod.txt**: Text file with core project information.

### Core Modules

#### `mdao/`
- **SweepConfig.py**: Configures parameter sweeps for multi-disciplinary analysis.
- **TradeStudy.py**: Facilitates trade studies for design optimization.

#### `mission/`
- **MissionPlanner.py**: Handles mission planning and execution.

#### `plume/`
- **IsentropicExpansion.py**: Calculates properties of isentropic expansions.
- **RarefiedPlumeGasKinetics.py**: Models the kinetics of rarefied plume gases.

#### `rpod/`
- **JetFiringHistory.py**: Manages and analyzes jet firing data.
- **RPOD.py**: Core logic for rendezvous and proximity operations.
- **header.py**: Contains shared constants and configurations for the `rpod` module.

#### `util/`
- **io/file_print.py**: Utility for formatted output and file handling.
- **stl/transform_stl.json**: JSON configuration for STL transformations.
- **stl/transform_stl.py**: Script for transforming STL files (scaling, translation).

#### `vehicle/`
- **LogisticsModule.py**: Handles logistics of vehicle operations.
- **TargetVehicle.py**: Defines the behavior and characteristics of target vehicles.
- **Vehicle.py**: Base class for generic vehicle simulations.
- **VisitingVehicle.py**: Defines visiting vehicle behavior.

## Usage

### Running the Scripts
Scripts are designed to perform specific tasks such as mission planning, vehicle modeling, or plume analysis. Use appropriate test cases or configurations to execute them.

### Transforming STL Files
To apply transformations to STL files:
1. Edit the `util/stl/transform_stl.json` configuration file with desired scaling and translation parameters.
2. Run the transformation script:
   ```bash
   python util/stl/transform_stl.py util/stl/transform_stl.json
   ```

### Testing
Run tests using the `config_test.py` or dedicated test cases provided for each module.

## Logging

We use Python's logging across modules.

- Get a logger:
  - from pyrpod.logging_utils import get_logger
  - logger = get_logger("pyrpod.<package>.<module>")
- Default level: INFO. Override via env var (PowerShell):
  - $env:PYRPOD_LOG_LEVEL = "DEBUG"
- Customize format via env var:
  - $env:PYRPOD_LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"

Example:

```python
from pyrpod.logging_utils import get_logger
logger = get_logger("pyrpod.example")
logger.info("Hello from PyRPOD")
```

## Adding New Modules
1. Place new scripts in the relevant directory.
2. Ensure the module is well-documented with clear docstrings.
3. Add test cases to validate the new functionality.
4. Follow naming conventions and ensure compatibility with existing modules.
