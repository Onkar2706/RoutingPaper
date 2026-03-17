import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def count_zone_switches(route, zones):
    switches = 0
    for i in range(len(route) - 1):
        if zones[route[i]] != zones[route[i+1]]:
            switches += 1
    return switches

def generate_locations(n: int, seed: int = 42) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(0, 100, size=(n, 2))


def assign_zones(locations: np.ndarray) -> list[int]:
    zones = []
    for x, y in locations:
        if x < 50 and y < 50:
            zones.append(0)
        elif x >= 50 and y < 50:
            zones.append(1)
        elif x < 50 and y >= 50:
            zones.append(2)
        else:
            zones.append(3)
    return zones


def euclidean_distance(a: np.ndarray, b: np.ndarray) -> int:
    return int(np.linalg.norm(a - b))


def create_constraint_matrix(
    locations: np.ndarray,
    zones: list[int],
    zone_penalty: int = 15,
    sequence_penalty: int = 10,
) -> list[list[int]]:
    n = len(locations)
    matrix = [[0] * n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i == j:
                continue

            base_cost = euclidean_distance(locations[i], locations[j])
            penalty = 0

            # Penalize switching zones
            if zones[i] != zones[j]:
                penalty += zone_penalty

            # Simple sequence penalty:
            # discourage going "backward" by index
            if j < i:
                penalty += sequence_penalty

            matrix[i][j] = base_cost + penalty

    return matrix




def solve_tsp(cost_matrix: list[list[int]]) -> tuple[int, list[int]]:
    n = len(cost_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def cost_callback(from_index: int, to_index: int) -> int:
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return cost_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(cost_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.seconds = 5

    solution = routing.SolveWithParameters(search_parameters)
    if solution is None:
        raise RuntimeError("No solution found")

    index = routing.Start(0)
    route = []
    total_cost = 0

    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        route.append(node)
        next_index = solution.Value(routing.NextVar(index))
        total_cost += routing.GetArcCostForVehicle(index, next_index, 0)
        index = next_index

    route.append(manager.IndexToNode(index))
    return total_cost, route


if __name__ == "__main__":
    locations = generate_locations(50)
    zones = assign_zones(locations)
    cost_matrix = create_constraint_matrix(locations, zones)

    cost, route = solve_tsp(cost_matrix)

    print("Constraint-aware cost:", cost)
    print("Route:", route)
    print("Zones:", zones)