# Constraint-Aware Routing Optimization

This repository contains the implementation of a constraint-aware routing framework for logistics optimization.

## Problem
Traditional routing models minimize distance but ignore operational constraints such as zone transitions.

## Contribution
- Introduced penalty-based cost function
- Reduced zone switching in routes
- Maintained similar runtime performance

## Structure
- baseline_solver.py → standard TSP solver
- constraint_solver.py → constraint-aware model
- run_experiment.py → experiment pipeline
- results.csv → experiment results

## How to Run

```bash
pip install ortools
python run_experiment.py
