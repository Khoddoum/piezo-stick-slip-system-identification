# Piezo-Stick-Slip System Identification

## About the Project
This repository contains the MATLAB source code developed as part of academic "Thesis" research focusing on the modeling and semi-analytical solution of stick-slip piezoelectric motors[cite: 4]. Stick-slip actuators rely on friction-drive mechanisms that introduce inherent nonlinearities, such as piecewise-smooth dynamics and Coulomb friction[cite: 4, 11]. Accurately identifying their mechanical parameters is highly complex. This project addresses that challenge by performing robust system identification across multiple experimental datasets using an optimization approach.

## Key Features
*   **Multi-Dataset Optimization:** Utilizes MATLAB's `fmincon` interior-point algorithm to simultaneously train the model on diverse operating conditions (e.g., varying voltages or duty ratios)[cite: 2, 7, 13].
*   **Custom Numerical Solvers:** Integrates a specialized 4th-order Runge-Kutta (RK4) solver tailored to handle discontinuous dynamics[cite: 9, 18].
*   **Automated Validation:** Features built-in routines to evaluate model robustness against unseen data by tracking displacement (MAE) and steady-state velocity errors[cite: 9, 18].
*   **Advanced Modeling Foundation:** Serves as the numerical baseline for further semi-analytical investigations utilizing the Harmonic Balance Method (HBM) and Time-Domain Collocation (TDC)[cite: 4, 9].

## Prerequisites
*   **MATLAB:** Version R2022a or newer is highly recommended.
*   **Toolboxes:** Optimization Toolbox is strictly required to execute the `fmincon` functions[cite: 7].
*   **Data Files:** Requires experimental `.mat` datasets formatted properly (e.g., `DR99f1V100.mat`)[cite: 8]. 

## Usage
1. Clone this repository to your local machine.
2. Place all experimental `.mat` files in the root directory alongside the main script[cite: 8].
3. Open `Main_Identification.m` in MATLAB.
4. Run the script section-by-section. The code will automatically evaluate the training fit, validate the identified parameters, and output the final optimal values directly to your workspace[cite: 13, 17].
5.images/V100 V.png
