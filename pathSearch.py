"""
経路探索などを行う
"""
import math
import networkx as nx

#クリックされた点を含む最小の矩形領域
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

#最短経路
def shortestPath(p1, p2, link, length):
    #最近傍ノードを取得
    p1 = nearestNode(p1, link)
    p2 = nearestNode(p2, link)

    #networkxのグラフを生成
    edges = []
    for i in range(len(link)):
        edges.append((str(link[i][0]), str(link[i][1]), length[i]))
    G = nx.Graph()
    G.add_weighted_edges_from(edges)
    connectGraph(G)

    #最短経路探索
    path_str = nx.dijkstra_path(G, str(p1), str(p2))

    #結果をstrから戻して返却
    path = []
    for line in path_str:
        path.append([float(x) for x in line.strip('[]').split(',')])
    return path

#最小全域木
def MST(link, length):
    #networkxのグラフを生成
    edges = []
    for i in range(len(link)):
        edges.append((str(link[i][0]), str(link[i][1]), length[i]))
    G = nx.Graph()
    G.add_weighted_edges_from(edges)
    connectGraph(G)

    #最小全域木を構成
    t = nx.minimum_spanning_tree(G)
    path = []
    for edge in t.edges():
        start = [float(x) for x in edge[0].strip('[]').split(',')]
        end = [float(x) for x in edge[1].strip('[]').split(',')]
        path.append([start, end])
    return path

#シュタイナー木
def steiner(points, link, length):
    #ターミナルノード
    terminal = []
    for p in points:
        terminal.append(str(nearestNode(p, link)))

    #networkxのグラフを生成
    edges = []
    for i in range(len(link)):
        edges.append((str(link[i][0]), str(link[i][1]), length[i]))
    G = nx.Graph()
    G.add_weighted_edges_from(edges)
    connectGraph(G)

    #シュタイナー木を構成
    t = nx.algorithms.approximation.steinertree.steiner_tree(G, terminal_nodes=terminal, method='mehlhorn')
    path = []
    for edge in t.edges():
        start = [float(x) for x in edge[0].strip('[]').split(',')]
        end = [float(x) for x in edge[1].strip('[]').split(',')]
        path.append([start, end])
    return path

#巡回経路
def traveling(points, link, length):
    #通るポイント(都市)
    positions = []
    for p in points:
        positions.append(str(nearestNode(p, link)))

    #巡回セールスマン問題のためのグラフ
    G = nx.Graph()
    G.add_nodes_from(positions)

    #都市間の最短経路を求めるためのグラフ
    G_temp = nx.Graph()
    edges = []
    for i in range(len(link)):
        edges.append((str(link[i][0]), str(link[i][1]), length[i]))
    G_temp.add_weighted_edges_from(edges)

    #都市間の最短経路を求めて，Gのエッジとする
    for u in positions:
        for v in positions:
            if positions.index(u) < positions.index(v):
                G.add_edge(u, v, weight=nx.dijkstra_path_length(G_temp, u, v))
    
    #巡回セールスマン問題を解く
    tsp = list(nx.algorithms.approximation.traveling_salesman_problem(G))

    #巡回順に最短経路を求めて返却
    paths = []
    length = 0
    for i in range(len(tsp)-1):
        path_str = nx.dijkstra_path(G_temp, tsp[i], tsp[i+1])
        length += nx.dijkstra_path_length(G_temp, tsp[i], tsp[i+1])
        for line in path_str:
            paths.append([float(x) for x in line.strip('[]').split(',')])
    return paths, length