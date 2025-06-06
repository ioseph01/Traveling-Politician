# -*- coding: utf-8 -*-
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math
import json


time = int(input("Enter time to process (default is 60): "))

with open("data.json") as f:
    capitals = json.load(f)
    

"""

Data in format 
[
{ "state" : '' , "capital" : '' , "lat" : '' , "long" : '' },
{ "state" : '' , "capital" : '' , "lat" : '' , "long" : '' },
{ "state" : '' , "capital" : '' , "lat" : '' , "long" : '' }
]
With the first state being the starting point and the last being the ending point

"""

# Haversine distance function (in kilometers)
def haversine(lat1, lon1, lat2, lon2):
    R = 6371
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))

# Build distance matrix
n = len(capitals)
distance_matrix = [
    [float(haversine(a["lat"], a["long"], b["lat"], b["long"])) for b in capitals]
    for a in capitals
]

# Indexes
start_index = 0  # Des Moines
end_index = len(capitals) - 1  # Washington, D.C.

# Create Routing Index Manager
manager = pywrapcp.RoutingIndexManager(n, 1, [start_index], [end_index])



# Routing model
routing = pywrapcp.RoutingModel(manager)

# Distance callback
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Search parameters
params = pywrapcp.DefaultRoutingSearchParameters()
params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
params.time_limit.seconds = time  # Try 60–300 for best results

# Solve
print("Solving...")
solution = routing.SolveWithParameters(params)

# Print route
if solution:
    index = routing.Start(0)
    print("Optimal route:")
    total_distance = 0
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        print(f"{capitals[node_index]['capital']} ({capitals[node_index]['state']})")
        next_index = solution.Value(routing.NextVar(index))
        total_distance += distance_callback(index, next_index)
        index = next_index
    # Final stop: DC
    dc = capitals[end_index]
    print(f"{dc['capital']} ({dc['state']})")
    print(f"\nTotal travel distance: {int(total_distance):,} km")
else:
    print("No solution found.")


