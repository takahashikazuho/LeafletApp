"""
経路探索などを行う
"""
import math
import networkx as nx
import pyproj
import db

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

#巡回経路
def travelingPath(points, link, length):
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
                G.add_edge(u, v, weight=nx.dijkstra_path_length(G_temp, u, v))
    
    #巡回セールスマン問題を解く
    tsp = list(nx.algorithms.approximation.traveling_salesman_problem(G))

    #巡回順に最短経路を求めて返却
    path = []
    length = 0
    for i in range(len(tsp)-1):
        path_str = nx.dijkstra_path(G_temp, tsp[i], tsp[i+1])
        length += nx.dijkstra_path_length(G_temp, tsp[i], tsp[i+1])
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    return path, length, tsp

#経由点決定(あまのさん)
def viterbi_ver1(tsp, candidates, G):
    positions_SRP = [tsp[0]]
    for i in range(len(tsp)-1):
        dist_min = float('inf')
        node_min = ""
        for node in candidates[i+1]:
            if node in list(G.nodes):
                dist = nx.dijkstra_path_length(G, positions_SRP[i], node)
                if dist < dist_min:
                    dist_min = dist
                    node_min = node
        positions_SRP.append(node_min)
    return positions_SRP

#経由点決定(３点間)
def viterbi_ver2(tsp, candidates, G):
    positions_SRP = [tsp[0]]
    for i in range(len(tsp)):
        dist_min = float('inf')
        node_min = ""
        #if n == len(tsp)-1:
        for node in candidates[i+1]:
            dist = nx.dijkstra_path_length(G, positions_SRP[i], node) + nx.dijkstra_path_length(G, tsp[i+1], node) + nx.dijkstra_path_length(G, tsp[i+2], node)
            if dist < dist_min:
                    dist_min = dist
                    node_min = node
        positions_SRP.append(node_min)

        #if n == len(tsp):
    l = len(tsp)-2
    return positions_SRP


#相乗り経路
def sharedRidePath(points, link, length, moveDist):
    #pointsとpositionsを辞書にして，tspとpointsを結びつける
    #巡回順になったpointsを返却する

    #通るポイント(都市)
    point_dict = {}
    positions = []
    for p in points:
        node = str(nearestNode(p, link))
        positions.append(node)
        point_dict[node] = p

    #巡回セールスマン問題のためのグラフ
    G_temp = nx.Graph()
    G_temp.add_nodes_from(positions)

    #都市間の最短経路を求めるためのグラフ
    G = linkToGraph(link, length)
    connectGraph(G)

    #都市間の最短経路を求めて，Gのエッジとする
    for u in positions:
        for v in positions:
            if positions.index(u) < positions.index(v):
                G_temp.add_edge(u, v, weight=nx.dijkstra_path_length(G, u, v))
    
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
            if nx.dijkstra_path_length(G_temp, p, node) <= moveDist/1000:
                candidate.append(node)
        candidates.append(candidate)

    #順に候補点から経由点を決定
    positions_SRP = [tsp[0]]
    for i in range(len(tsp)-1):
        dist_min = float('inf')
        node_min = ""
        for node in candidates[i+1]:
            if node in list(G.nodes):
                dist = nx.dijkstra_path_length(G, positions_SRP[i], node)
                if dist < dist_min:
                    dist_min = dist
                    node_min = node
        positions_SRP.append(node_min)
    
    #巡回順に最短経路を求めて返却
    positions_SRP.append(positions_SRP[0])
    path = []
    length_SRP = 0
    for i in range(len(positions_SRP)-1):
        path_str = nx.dijkstra_path(G, positions_SRP[i], positions_SRP[i+1])
        length_SRP += nx.dijkstra_path_length(G, positions_SRP[i], positions_SRP[i+1])
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    positions_SRP.pop()

    #巡回順のクリックされた点の座標
    points_SRP = []
    for p in tsp:
        points_SRP.append(point_dict[p])

    #各移動先までの経路
    path_positions = []
    positions_SRP_a = []
    for i in range(len(positions_SRP)):
        positions_SRP_a.append(positions_SRP[i].strip("[]").split(","))
        point = str(nearestNode(points_SRP[i], link))
        if point != positions_SRP[i]:
            path_str = nx.dijkstra_path(G, point, positions_SRP[i])
            path_temp = []
            for line in path_str:
                path_temp.append([float(x) for x in line.strip('[]').split(',')])
            path_positions.append(path_temp)

    return path, length_SRP, points_SRP, positions_SRP_a, path_positions