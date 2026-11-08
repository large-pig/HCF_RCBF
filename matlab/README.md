# MATLAB simulation

This directory contains the MATLAB source for comparing HCF, RCBF, and
ACBF under three disturbance scenarios. The model, controller
parameters, initial states, and disturbance profiles are defined in
the source files.

## Run the study

Open this directory in MATLAB and run:

```matlab
Run_Analysis
```

This runs the initial checks, recomputes all nine trajectories, and opens
the six comparison figures. The program writes its results to a new
`results_YYYYMMDD_HHMMSS` directory and copies the trajectory and metric
CSVs to the source directory for convenience.

MATLAB with Optimization Toolbox is required for the ACBF baseline,
which calls `quadprog`.

To run the initial checks independently:

```matlab
selftest_three_scenarios
```

To recompute the trajectories and metrics without opening the six
comparison figure windows:

```matlab
[metrics, outdir] = run_three_scenarios;
```
