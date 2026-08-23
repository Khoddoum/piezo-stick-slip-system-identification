# Piezo-Stick-Slip System Identification

## About the Project
This repository contains the MATLAB source code developed as part of my "Thesis" research focusing on the modeling and semi-analytical solution of stick-slip piezoelectric motors[cite: 1, 4]. Actuators relying on friction-drive mechanisms introduce inherent nonlinearities, making parameter identification a highly complex task[cite: 1, 4]. This project addresses that challenge by performing robust system identification across multiple experimental datasets using an optimization approach to accurately estimate physical parameters[cite: 4, 13].

## Key Features
*   **Multi-Dataset Optimization:** Utilizes MATLAB's `fmincon` interior-point algorithm to simultaneously train the model on diverse operating conditions[cite: 1, 13].
*   **Custom Numerical Solvers:** Integrates a specialized 4th-order Runge-Kutta (RK4) solver tailored to handle discontinuous dynamics[cite: 1, 4].
*   **Automated Validation:** Evaluates model robustness against unseen data by tracking displacement Mean Absolute Error (MAE) and steady-state velocity errors[cite: 1, 4].

## Prerequisites
*   **MATLAB:** Recommended R2022a or newer.
*   **Toolboxes:** Optimization Toolbox is strictly required to execute the optimization functions[cite: 1, 4].
*   **Data Files:** Requires experimental `.mat` datasets placed in the root directory[cite: 4, 8].

## Usage
1. Clone this repository to your local machine.
2. Place all experimental `.mat` files in the root directory alongside the main script[cite: 4].
3. Run the script section-by-section to evaluate the training fit, validate the identified parameters, and extract the final optimal values[cite: 4, 13].

## Results
Below is the training fit result demonstrating the identified model's performance compared to the experimental data at 100V:

![Training Result at 100V](V100 V.png)
