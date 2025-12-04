import requests
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

class VRP:
    def __init__(self, coords, vehicle_capacities, demands):
        self.coords = coords
        self.vehicle_capacities = vehicle_capacities
        self.demands = demands
    
    # Method - 1
    def _get_distance_matrix(self):
        if not self.coords:
            raise ValueError("Coordinates list is empty")

        base_url = "http://router.project-osrm.org/table/v1/driving/"
        loc_str = ";".join([f"{lon},{lat}" for lon, lat in self.coords])
        url = base_url + loc_str + "?annotations=distance,duration"
        r = requests.get(url)
        # r.raise_for_status() # raise exception if any case
        data = r.json()

        if "distances" in data and "durations" in data:
            return data["durations"], data["distances"]
        elif "durations" in data:
            return data["durations"], None
        else:
            raise ValueError("Unexpected OSRM response: " + str(data))

    
    # Method - 2
    def _create_data_model(self):
        # try:
        duration_matrix, distance_matrix = self._get_distance_matrix()
        # except Exception as e:
        #     raise RuntimeError(f"Failed to create distance matrix: {e}")
        
        # matrix = distance_matrix if distance_matrix is not None else duration_matrix
        
        # if not matrix:
        #     raise ValueError("Distance/duration matrix is empty")

        data = {}
        data['distance_matrix'] = distance_matrix
        data['distance_matrix'] = [[int(d) for d in row] for row in data['distance_matrix']]
        # data['distance_matrix'] = [[int(round(d)) for d in row] for row in matrix]
        data['demands'] = self.demands
        data['vehicle_capacities'] = self.vehicle_capacities
        data['num_vehicles'] = len(self.vehicle_capacities)
        data['depot'] = 0
        return data
    
    # Method - 3
    def _print_solution(self, data, manager, routing, solution):
        total_distance = 0
        total_load = 0
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            route_load = 0
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_load += data['demands'][node_index]
                plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                    route_load)
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            plan_output += 'Load of the route: {}\n'.format(route_load)
            print(plan_output)
            total_distance += route_distance
            total_load += route_load
        print('Total distance of all routes: {}m'.format(total_distance))
        print('Total load of all routes: {}'.format(total_load))

    
    # Method - 4
    def _solution_to_json(self, data, manager, routing, solution):
        result = {
            "routes": [],
            "total_distance": 0,
            "total_load": 0
        }

        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            route_info = {
                "vehicle_id": vehicle_id,
                "stops": [],
                "distance": 0,
                "load": 0
            }

            route_distance = 0
            route_load = 0

            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_load += data['demands'][node_index]

                route_info["stops"].append({
                    "node": node_index,
                    "load_after_visit": route_load
                })

                prev_index = index
                index = solution.Value(routing.NextVar(index))

                route_distance += routing.GetArcCostForVehicle(
                    prev_index, index, vehicle_id
                )

            # Add the final depot stop
            route_info["stops"].append({
                "node": manager.IndexToNode(index),
                "load_after_visit": route_load
            })

            route_info["distance"] = route_distance
            route_info["load"] = route_load

            if route_load > 0:
                result["routes"].append(route_info)
                result["total_distance"] += route_distance
                result["total_load"] += route_load

        return result
    

    def CVRP(self):
        # Instantiate the data problem.
        data = self._create_data_model()

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                            data['num_vehicles'], data['depot'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)


        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = manager.IndexToNode(from_index)
            return data['demands'][from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(
            demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            data['vehicle_capacities'],  # vehicle maximum capacities
            True,  # start cumul to zero
            'Capacity')

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.FromSeconds(1)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            self._print_solution(data, manager, routing, solution)
            return self._solution_to_json(data, manager, routing, solution)

        return {"error": "No solution found"}
    


