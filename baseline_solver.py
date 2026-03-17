import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from constraint_solver import count_zone_switches


def generate_locations(n: int, seed: int = 42) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(0, 100, size=(n, 2))


def euclidean_distance(a: np.ndarray, b: np.ndarray) -> int:
    return int(np.linalg.norm(a - b))

def count_zone_switches(route, zones):
    switches = 0
    for i in range(len(route) - 1):
        if zones[route[i]] != zones[route[i+1]]:
            switches += 1
    return switches
def count_zone_switches(route, zones):
    switches = 0
    for i in range(len(route) - 1):
        if zones[route[i]] != zones[route[i+1]]:
            switches += 1
    return switches

def create_distance_matrix(locations: np.ndarray) -> list[list[int]]:
    n = len(locations)
    matrix = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                matrix[i][j] = euclidean_distance(locations[i], locations[j])
    return matrix


def solve_tsp(distance_matrix: list[list[int]]) -> tuple[int, list[int]]:
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index: int, to_index: int) -> int:
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
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
    distance_matrix = create_distance_matrix(locations)
    cost, route = solve_tsp(distance_matrix)

    print("Baseline cost:", cost)
    print("Route:", route)