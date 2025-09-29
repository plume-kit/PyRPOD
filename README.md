# PyRPOD
Python Based Analysis of Rendezvous, Proximity, Operation, and Docking Maneuvers (PyRPOD)
![PyRPOD Logo](data/img/PyRPOD.png)

## Overview
This code simulates jet firing histories, vehicle dynamics, and plume impingement interactions to study RPOD (Rendezvous, Proximity Operations, and Docking) maneuvers relevant to active space-station operations.

The objective is to enable rapid analysis of early vehicle designs to assess compliance with competing mission requirements and to explore design spaces for iteration toward an optimal design. Ideally, this tool aims to increase operational safety and decrease design costs of active space-station operation. The project features parameterized simulation cases, fuel efficiency calculations, plume-surface interaction modeling, and automated transformations between STL and VTK formats to support flexible data visualization.

PyRPOD utilizies scientific libraries such as NumPy, SciPy, Matplotlib, and SymPy, with outputs in STL and VTK formats suitable for visualization in ParaView. Modular testing and validation cases ensures accuracy code resiliency for future developments. The target audience for this code is researchers and engineers in aerospace engineering and computational modeling.

## Directory Structure

- `case/`: Contains specific case configurations, results, and model transformations.
- `data/`: Stores raw data files used across cases, including flight plans, STL models, and test definitions.
- `pyrpod/`: Python modules for running and managing simulations (main source code).
- `tests/`: Unit, integration, and verification tests for the project.
- `validation/`: Validation and Verification Tests of physical models used.
- `requirements.txt`: Lists project dependencies for environment setup.

## Getting Started
1. Clone the repository:
   ```bash
   git clone https://github.com/plume-kit/PyRPOD
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Run  all test cases.
   ```bash
   cd tests/
   python3 test_all.py
   ```

3. Run individual test cases (Not ideal, I know).
   ```bash
   cd tests/
   mv <test_module>/test_case.py .
   python3 test_case.py
   ```