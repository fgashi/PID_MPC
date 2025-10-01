
# README

## Project Overview
This repository accompanies the paper published at **MED’24** (Mediterranean Conference on Control and Automation 2024). It includes all scripts used for simulating the system model, processing data, and generating the results presented in the paper.

## Folder Structure
```
.
├── Robotino_example/          # Python script for optimal reference generation
│   └── robotino_mpc.py
├── Robotino_example_K/        # Python script for Robotino with PID as decision variables
│   └── robotino_mpc.py
├── overleaf_files/            # Processed results integrated with Overleaf (Graphics directory)
├── README.md                  # This file
└── [other supporting files]   # Optional additional resources or scripts
```

## Contents
- **Python scripts**: Simulate the system model, collect results, and store them as `.txt` files.  
- **Processed results**: Output data are automatically uploaded to `overleaf_files`, which is integrated into the Overleaf project (mainly under the `Graphics` directory) via git. Most figures are plotted directly on Overleaf.

> ⚠️ Due to figure rendering on Overleaf, **local compilation** is strongly recommended to avoid prolonged PDF compilation times.

## Dependencies
To reproduce the results, ensure the following dependencies are installed:
- **CasADi v3.5**  
- **Ipopt**  
- **MA57** (linear solver from the Harwell Subroutine Library)  

It is recommended to use a Python environment (e.g., `venv` or `conda`) for managing these dependencies.

## How to Run
- For **optimal reference generation**, run:
  ```bash
  python Robotino_example/robotino_mpc.py
  ```

- For the **Robotino case with PID parameters as decision variables**, run:
  ```bash
  python Robotino_example_K/robotino_mpc.py
  ```
  Here, `K` stands for `PID(k)`, indicating that the PID parameters vary over time step *k*.

## Citation
If you use this repository or its results, please cite the following paper:

```bibtex
@INPROCEEDINGS{10566273,
  author={Gashi, Fatos and Abuibaid, Khalil and Ruskowski, Martin and Wagner, Achim},
  booktitle={2024 32nd Mediterranean Conference on Control and Automation (MED)}, 
  title={Model Predictive Control Based Reference Generation for Optimal Proportional Integral Derivative Control}, 
  year={2024},
  pages={518-524},
  keywords={PI control;Automation;PD control;Tuning;Optimization;Predictive control},
  doi={10.1109/MED61351.2024.10566273}
}
```
