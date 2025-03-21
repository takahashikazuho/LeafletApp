"""
経路探索などを行う
"""
import math
import networkx as nx
import pyproj
import db
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

#クリックされた点郡を含む最小の矩形領域
def rectangleArea(points):
    min_lat = float('inf')
    max_lat = float('-inf')
    min_lng = float('inf')
    max_lng = float('-inf')

    for point in points:
        lat, lng = point[0], point[1]
        min_lat = min(min_lat, lat)
        max_lat = max(max_lat, lat)
        min_lng = min(min_lng, lng)
        max_lng = max(max_lng, lng)

    return max_lat, min_lng, min_lat, max_lng

#ある点の周囲の矩形領域
#半径rの円に接する正方形の左上と右下の点
def aroundRectagleArea(y, x, r):
    grs80 = pyproj.Geod(ellps='GRS80')
    x1, y1, _back1 = grs80.fwd(float(y), float(x), 135, r * math.sqrt(2))
    x2, y2, _back2 = grs80.fwd(float(y), float(x), 315, r * math.sqrt(2))
    return y1, x1, y2, x2

#座標の最近傍ノードを取得
def nearestNode(p, link):
    dist = float('inf')
    nearestNode = None
    for line in link:
        for point in line:
            d = math.sqrt((p[0] - point[0])**2 + (p[1] - point[1])**2)
            if d < dist:
                dist = d
                nearestNode = point
    return nearestNode

#Gを連結グラフにする
def connectGraph(G):
    if not nx.algorithms.components.is_connected(G):
        components = list(nx.algorithms.components.connected_components(G))
        for i in range(len(components) - 1):
            if not nx.algorithms.components.is_connected(G):
                component1 = components[i]
                component2 = components[i + 1]
                node1 = next(iter(component1))
                node2 = next(iter(component2))
                G.add_weighted_edges_from([(node1, node2, float('inf'))])

#リンクからグラフ生成
def linkToGraph(link, length):
    edges = []
    for i in range(len(link)):
        edges.append((str(link[i][0]), str(link[i][1]), length[i]))
    G = nx.Graph()
    G.add_weighted_edges_from(edges)
    return G

#最短経路長が辞書になかったらダイクストラで計算
def len_SP(G, node1, node2, len_dic):
    try:
        return len_dic[node1][node2]
    except KeyError:
        return nx.dijkstra_path_length(G, node1, node2)
    
#巡回経路
def travelingPath(points, link, length, value, len_dic):
    #通るポイント(都市)
    positions = []
    for p in points:
        positions.append(str(nearestNode(p, link)))

    #巡回セールスマン問題のためのグラフ
    G = nx.Graph()
    G.add_nodes_from(positions)

    #都市間の最短経路を求めるためのグラフ
    G_temp = linkToGraph(link, length)
    connectGraph(G_temp)

    #都市間の最短経路を求めて，Gのエッジとする
    for u in positions:
        for v in positions:
            if positions.index(u) < positions.index(v):
                G.add_edge(u, v, weight=len_SP(G_temp, u, v, len_dic))
    
    #巡回セールスマン問題を解く
    if value == "type1":
        tsp = list(nx.algorithms.approximation.traveling_salesman_problem(G))
    if value == "type2":
        init = list(nx.algorithms.approximation.traveling_salesman_problem(G))
        tsp = nx.algorithms.approximation.simulated_annealing_tsp(G, init)
    if value == "type3":
        tsp = nx.algorithms.approximation.simulated_annealing_tsp(G, "greedy")

    #巡回順に最短経路を求めて返却
    path = []
    length = 0
    for i in range(len(tsp)-1):
        path_str = nx.dijkstra_path(G_temp, tsp[i], tsp[i+1])
        length += len_SP(G_temp, tsp[i], tsp[i+1], len_dic)
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    return path, length, tsp

# #経由点決定(あまのさん)
# def viterbi_ver1(tsp, candidates, len_dic, G):
#     #経由点集合
#     positions_SRP = []
#     #各候補点間の最短経路長を格納
#     path_length = {}
#     path_length[tsp[0]] = 0
#     path_backtrack = {}

#     #各候補点間の最短経路長を求める
#     candidates[0] = [tsp[0]]
#     for i in range(len(candidates)):
#         n = i + 1
#         if i == len(candidates)-1:
#             n = 0
#         for node in candidates[n]: 
#             dist_min = float('inf')
#             node_min = ""
#             for node_prev in candidates[i]:
#                 dist = len_SP(G, node, node_prev, len_dic) + path_length[node_prev]
#                 if dist < dist_min:
#                     dist_min = dist
#                     node_min = node_prev
#             path_length[node] = dist_min
#             if node in path_backtrack:
#                 path_backtrack[node][n] = node_min
#             else:
#                 path_backtrack[node] = {n:node_min}
            
#     #各候補点間の最短経路を遡ることにより最短経路を得る
#     node = path_backtrack[tsp[0]][0]
#     for i in reversed(range(len(candidates))):
#         positions_SRP.insert(0, node)
#         node = path_backtrack[node][i]

#     return positions_SRP

#経由点決定(乗客移動距離考慮)
def viterbi_ver2(tsp, candidates, len_dic, G, alpha=0.01):
    #経由点集合
    positions_SRP = []
    #各候補点間の最短経路長を格納
    path_length = {}
    path_length[tsp[0]] = 0
    path_backtrack = {}

    #各候補点間の最短経路長を求める
    candidates[0] = [tsp[0]]
    for i in range(len(candidates)):
        n = i + 1
        r = 0
        if i == len(candidates)-1:
            n = 0
        for node in candidates[n]: 
            dist_min = float('inf')
            node_min = ""
            for node_prev in candidates[i]:
                a = len_SP(G, node, node_prev, len_dic)
                b = path_length[node_prev]
                c = len_SP(G, node, tsp[n], len_dic) * alpha
                dist = c + b + a
                if dist < dist_min:
                    dist_min = dist
                    node_min = node_prev
            path_length[node] = dist_min
            if node in path_backtrack:
                path_backtrack[node][n] = node_min
            else:
                path_backtrack[node] = {n:node_min}
            
    #各候補点間の最短経路を遡ることにより最短経路を得る
    node = path_backtrack[tsp[0]][0]
    for i in reversed(range(len(candidates))):
        positions_SRP.insert(0, node)
        node = path_backtrack[node][i]

    #移動先の点と移動前の点を対応
    points_move_dic = {}
    for i in range(len(tsp)):
        if positions_SRP[i] in points_move_dic:
            points_move_dic[positions_SRP[i]].append(tsp[i])
        else:
            points_move_dic[positions_SRP[i]] = [tsp[i]]
    # print(positions_SRP)
    return positions_SRP, points_move_dic

#巡回経路(2-opt)
def two_opt(path, len_dic, G):
    size = len(path)
    while True:
        count = 0
        for i in range(size - 2):
            i1 = i + 1
            for j in range(i + 2, size):
                if j == size - 1:
                    j1 = 0
                else:
                    j1 = j + 1
                if i != 0 or j1 != 0:
                    l1 = len_SP(G, path[i], path[i1], len_dic)
                    l2 = len_SP(G, path[j], path[j1], len_dic)
                    l3 = len_SP(G, path[i], path[j], len_dic)
                    l4 = len_SP(G, path[i1], path[j1], len_dic)
                    if l1 + l2 > l3 + l4:
                        # つなぎかえる
                        new_path = path[i1:j+1]
                        path[i1:j+1] = new_path[::-1]
                        count += 1
        if count == 0: break
    return path

#巡回経路(焼きなまし)
def annealing(path, len_dic, G):
    G_temp = nx.Graph()
    G_temp.add_nodes_from(path)
    for u in path:
        for v in path:
            if path.index(u) < path.index(v):
                G_temp.add_edge(u, v, weight=len_SP(G, u, v, len_dic))
    path.append(path[0])
    positions_SRP = nx.algorithms.approximation.simulated_annealing_tsp(G_temp, path)
    positions_SRP.pop()
    return positions_SRP

#相乗り経路
def sharedRidePath(points, link, length, moveDist, value, len_dic):
    #通るポイント(都市)
    point_dic = {}
    positions = []
    for p in points:
        node = str(nearestNode(p, link))
        positions.append(node)
        point_dic[node] = p

    #巡回セールスマン問題のためのグラフ
    G_temp = nx.Graph()
    G_temp.add_nodes_from(positions)

    #都市間の最短経路を求めるためのグラフ
    G = linkToGraph(link, length)
    connectGraph(G)

    #都市間の最短経路を求めて，G_tempのエッジとする
    for u in positions:
        for v in positions:
            if positions.index(u) < positions.index(v):
                G_temp.add_edge(u, v, weight=len_SP(G, u, v, len_dic))
    
    #巡回セールスマン問題を解く
    tsp = list(nx.algorithms.approximation.traveling_salesman_problem(G_temp))

    #乗客の移動候補ノードを取得
    tsp.pop()
    candidates = []
    for p in tsp:
        point = p.strip("[]").split(",")
        y1, x1, y2, x2 = aroundRectagleArea(point[1], point[0], moveDist)
        link_temp, length_temp = db.getRectangleRoadData(y1, x1, y2, x2)
        G_temp = linkToGraph(link_temp, length_temp)
        connectGraph(G_temp)
        candidate = [p]
        for node in list(G_temp.nodes):
            if node in G.nodes:
                if len_SP(G_temp, p, node, len_dic) <= moveDist/1000:
                    candidate.append(node)
        candidates.append(candidate)

    #順に候補点から経由点を決定
    if value == "type1":
        positions_SRP, points_move_dic = viterbi_ver2(tsp, candidates, len_dic, G)
    if value == "type2":
        positions_SRP, points_move_dic = viterbi_ver2(tsp, candidates, len_dic, G)
        positions_SRP = two_opt(positions_SRP, len_dic, G)
    if value == "type3":
        positions_SRP, points_move_dic = viterbi_ver2(tsp, candidates, len_dic, G)
        positions_SRP = annealing(positions_SRP, len_dic, G)
    if value == "type4":
        positions_SRP, points_move_dic = viterbi_ver2(tsp, candidates, len_dic, G, 0)

    
    #巡回順に最短経路を求めて返却
    positions_SRP.append(positions_SRP[0])
    path = []
    length_SRP = 0
    for i in range(len(positions_SRP)-1):
        path_str = nx.dijkstra_path(G, positions_SRP[i], positions_SRP[i+1])
        length_SRP += len_SP(G, positions_SRP[i], positions_SRP[i+1], len_dic)
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    positions_SRP.pop()

    #巡回順のクリックされた点の座標
    points_SRP = []
    for i in range(len(positions_SRP)):
        p = points_move_dic[positions_SRP[i]]
        points_SRP.append(point_dic[p[0]])
        if len(p) > 1:
            points_move_dic[positions_SRP[i]].pop(0)

    #各移動先までの経路
    path_positions = []
    positions_SRP_a = []
    len_walk = 0
    for i in range(len(positions_SRP)):
        positions_SRP_a.append(positions_SRP[i].strip("[]").split(","))
        point = str(nearestNode(points_SRP[i], link))
        if point != positions_SRP[i]:
            path_str = nx.dijkstra_path(G, point, positions_SRP[i])
            len_walk += len_SP(G, point, positions_SRP[i], len_dic)
            path_temp = []
            for line in path_str:
                path_temp.append([float(x) for x in line.strip('[]').split(',')])
            path_positions.append(path_temp)

    # print(len(tsp))
    # print(len(candidates))
    # print(len(positions_SRP))

    return path, length_SRP, points_SRP, positions_SRP_a, path_positions, len_walk

#########################################################################################
def init_vehicle(G, Q, st, len_dic):
    d = {}
    pie = {}
    T = {}
    for node in G.nodes:
        d[node] = float('inf')
        pie[node] = None
        T[node] = None
    d[st] = 0
    for q in Q:
        d[st] += len_SP(G, q[0], st, len_dic) + len_SP(G, q[1], st, len_dic)
    T[st] = {(st, 0)}
    for q in Q:
        T[st].add((q[0],len_SP(G, q[0], st, len_dic)))
        T[st].add((q[1],len_SP(G, q[1], st, len_dic)))
    return d, pie, T

def ORIS(G, Q, st, en, moveDist, len_dic):
    d, pie, T = init_vehicle(G, Q, st, len_dic)

    return

#########################################################################################
def create_data_model(points, link, length, len_dic):
    G = linkToGraph(link, length)
    connectGraph(G)
    positions = []
    for p in points:
        positions.append(str(nearestNode(p, link)))

    data = {}
    DM = []
    for p in positions:
        d = []
        for q in positions:
            d.append(len_SP(G, p, q, len_dic))
        DM.append(d)

    data["distance_matrix"] = DM
    data["num_vehicles"] = 4
    data["depot"] = 0

    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Maximum of the route distances: {max_route_distance}m")

def mTSP(points, link, length, len_dic):
    data = create_data_model(points, link, length, len_dic)
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found !")
