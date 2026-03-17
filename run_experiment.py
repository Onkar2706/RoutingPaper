import time
import pandas as pd

from baseline_solver import generate_locations, create_distance_matrix, solve_tsp as solve_baseline
from constraint_solver import assign_zones, create_constraint_matrix, solve_tsp as solve_constraint
from constraint_solver import count_zone_switches
from baseline_solver import count_zone_switches


def run_once(n: int, seed: int = 42) -> dict:
    locations = generate_locations(n, seed=seed)
    zones = assign_zones(locations)

    baseline_matrix = create_distance_matrix(locations)
    t1 = time.time()
    baseline_cost, baseline_route = solve_baseline(baseline_matrix)
    baseline_time = time.time() - t1
    baseline_switches = count_zone_switches(baseline_route, zones)

    constraint_matrix = create_constraint_matrix(locations, zones)
    t2 = time.time()
    constraint_cost, constraint_route = solve_constraint(constraint_matrix)
    constraint_time = time.time() - t2
    constraint_switches = count_zone_switches(constraint_route, zones)

    return {
        "nodes": n,
        "baseline_cost": baseline_cost,
        "constraint_cost": constraint_cost,
        "baseline_switches": baseline_switches,
        "constraint_switches": constraint_switches,
        "baseline_time_sec": round(baseline_time, 4),
        "constraint_time_sec": round(constraint_time, 4),
    }

def main() -> None:
    sizes = [20, 50, 100]
    rows = []

    for n in sizes:
        result = run_once(n)
        rows.append(result)

    df = pd.DataFrame(rows)
    print(df)
    df.to_csv("results.csv", index=False)
    print("Saved results to results.csv")


if __name__ == "__main__":
    main()