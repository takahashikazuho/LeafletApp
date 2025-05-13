"""
経路探索などを行う
"""
import math
import networkx as nx
import pyproj
import db
import heapq
import itertools
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

def reorder_tsp_st_to_en(lst, st, en):
    n = len(lst)
    idx_st = lst.index(st)
    idx_en = lst.index(en)
    # stとenが隣接していない場合はエラー
    if abs(idx_st - idx_en) != 1:
        raise ValueError('stとenは隣接していません')
    # 巡回的にリストを回転
    if (idx_st + 1) % n == idx_en:
        # st, en の順
        # st ... en ... (巡回的) ... st
        # 出力: st, 残り逆順, en
        between = []
        # en+1から末尾→先頭からst-1まで
        i = (idx_en+1) % n
        while i != idx_st:
            between.append(lst[i])
            i = (i+1) % n
        return [st] + between[::-1] + [en]
    elif (idx_en + 1) % n == idx_st:
        # en, st の順
        between = []
        i = (idx_st+1) % n
        while i != idx_en:
            between.append(lst[i])
            i = (i+1) % n
        return [st] + between + [en]
    else:
        raise ValueError('隣接判定ロジックエラー')

def path_TSP(st, en, points, link, length, len_dic):
    #通るポイント(都市)
    positions = []
    for p in points:
        positions.append(str(nearestNode(p, link)))
    st = str(nearestNode(st, link))
    en = str(nearestNode(en, link))

    #巡回セールスマン問題のためのグラフ
    G = nx.Graph()
    for i in range(len(positions)):
        G.add_node(positions[i], index=i)
    G.add_node(st, index=111)
    G.add_node(en, index=999)
    # G.add_nodes_from(positions)

    #都市間の最短経路を求めるためのグラフ
    G_temp = linkToGraph(link, length)
    connectGraph(G_temp)

    #都市間の最短経路を求めて，Gのエッジとする
    for u in positions:
        for v in positions:
            if positions.index(u) < positions.index(v):
                if (u == st and v == en) or (u == en and v == st):
                    continue
                G.add_edge(u, v, weight=len_SP(G_temp, u, v, len_dic))

    #st,en間に0エッジを追加
    G.add_edge(st, en, weight=0.001)
    
    #巡回セールスマン問題を解く
    tsp = list(nx.algorithms.approximation.simulated_annealing_tsp(G, init_cycle="greedy"))
    tsp.pop()
    tsp = reorder_tsp_st_to_en(tsp, st, en)

    #巡回順に最短経路を求めて返却
    path = []
    length = 0
    for i in range(len(tsp)-1):
        path_str = nx.dijkstra_path(G_temp, tsp[i], tsp[i+1])
        length += len_SP(G_temp, tsp[i], tsp[i+1], len_dic)
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    return path, length

def path_TSP_full_search(st, en, points, link, length, len_dic):
    # 通るポイント（都市）のノード名リスト作成
    positions = []
    for p in points:
        positions.append(str(nearestNode(p, link)))

    st = str(nearestNode(st, link))
    en = str(nearestNode(en, link))
    
    # グラフ作成
    G = nx.Graph()
    for i, pos in enumerate(positions):
        G.add_node(pos, index=i)
    G.add_node(st, index=111)
    G.add_node(en, index=999)

    # 都市間距離グラフの準備
    G_temp = linkToGraph(link, length)
    connectGraph(G_temp)

    # 都市間の最短経路によるグラフ構築
    all_points = positions + [st, en]
    for u, v in itertools.combinations(all_points, 2):
        if (u == st and v == en) or (u == en and v == st):
            continue
        # len_SP: ノードuからvまでの最短パス長
        G.add_edge(u, v, weight=len_SP(G_temp, u, v, len_dic))

    # 集合
    keypoints = positions  # ハミルトン路で必須経由するべきノード

    # 始点・終点は固定したい場合
    # (順序 [st, ???..., en] に限定)
    best_order = None
    best_length = float('inf')
    
    # end ノードは経路の最後、start ノードは最初
    inner_points = [p for p in keypoints if p != st and p != en]
    for perm in itertools.permutations(inner_points):
        order = [st] + list(perm) + [en]
        total = 0
        feasible = True
        for i in range(len(order)-1):
            try:
                dist = nx.shortest_path_length(G, source=order[i], target=order[i+1], weight='weight')
            except nx.NetworkXNoPath:
                feasible = False
                break
            total += dist
        if feasible and total < best_length:
            best_length = total
            best_order = order
    
    # 巡回順で最短経路を復元
    all_path = []
    length_sum = 0
    if best_order is not None:
        for i in range(len(best_order)-1):
            path_str = nx.dijkstra_path(G_temp, best_order[i], best_order[i+1])
            length_sum += len_SP(G_temp, best_order[i], best_order[i+1], len_dic)
            for line in path_str:
                all_path.append([float(x) for x in line.strip('[]').split(',')])

    return all_path, length_sum

def path_TSP_greedy(st, en, points, link, length, len_dic):
    # 通るポイント（都市）のノード名リスト作成
    positions = []
    for p in points:
        positions.append(str(nearestNode(p, link)))
    st = str(nearestNode(st, link))
    en = str(nearestNode(en, link))

    # 都市ノード集合
    all_cities = set(positions)
    if st not in all_cities:
        all_cities.add(st)
    if en not in all_cities:
        all_cities.add(en)
    all_cities = list(all_cities)

    # 都市間距離グラフ
    G_temp = linkToGraph(link, length)
    connectGraph(G_temp)

    # ハミルトン路・近似解
    current = st
    visited = set([current])
    order = [current]

    # 出発地から各都市へ貪欲に
    while len(visited) < len(all_cities):
        if len(visited) == len(all_cities)-1 and en not in visited:
            nxt = en  # 残るは終点enのみならそこへ行く
        else:
            # 未訪問都市のうち最も近いもの
            candidates = [city for city in all_cities if city not in visited and city != en]
            if not candidates:  # enしか残ってない場合例外処理
                break
            # 距離計算（len_SPでコストを取得）
            distances = [len_SP(G_temp, current, city, len_dic) for city in candidates]
            nxt = candidates[distances.index(min(distances))]
        order.append(nxt)
        visited.add(nxt)
        current = nxt

    # 最後に必ずenが順路に入っていることを確認
    if order[-1] != en:
        order.append(en)

    # 巡回順に最短経路に変換
    path = []
    length_sum = 0
    for i in range(len(order)-1):
        path_str = nx.dijkstra_path(G_temp, order[i], order[i+1])
        length_sum += len_SP(G_temp, order[i], order[i+1], len_dic)
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    return path, length_sum

#########################################################################################
class Routing:
    def __init__(self, link, length):
        self.link = link
        self.length = length
        self.G = linkToGraph(self.link, self.length)
        connectGraph(self.G)
        self.len_dic = dict(nx.all_pairs_dijkstra_path_length(self.G))

    def SPC(self, s, t):
        try:
            return self.len_dic[s][t]
        except KeyError:
            return nx.dijkstra_path_length(self.G, s, t)

    def init_vehicle(self, Q, st, SPC_cache):
        d = {}
        pie = {}
        T = {}
        mark = {}
        stop = {}

        for node in self.G.nodes:
            d[node] = float('inf')
            pie[node] = None
            T[node] = set()
            mark[node] = False
            stop[node] = False

        d[st] = 0
        T[st].add((st, 0))

        for q in Q:
            d[st] += SPC_cache.get((q[0], st), 0) + SPC_cache.get((q[1], st), 0)

        for q in Q:
            T[st].add((q[0], SPC_cache.get((q[0], st), 0)))
            T[st].add((q[1], SPC_cache.get((q[1], st), 0)))

        return d, pie, T, mark, stop

    def relax_vehicle(self, u, v, c, st, Q, SPC_cache, d, pie, T, PQ):
        if u not in T or not any(item[0] == st for item in T[u]):
            return d, pie, T, PQ  # Vehicle cost not found for u

        vehicle_cost_pair = next(item for item in T[u] if item[0] == st)
        cv = vehicle_cost_pair[1]
        d_temp_v = cv + c
        T_temp_v = {(st, cv + c)}

        for si, di in Q:
            cs = None
            cd = None
            if u in T:
                si_pair = next((item for item in T[u] if item[0] == si), None)
                di_pair = next((item for item in T[u] if item[0] == di), None)
                if si_pair:
                    cs = si_pair[1]
                if di_pair:
                    cd = di_pair[1]

            cost_si_v = SPC_cache.get((si, v), float('inf'))
            cost_di_v = SPC_cache.get((di, v), float('inf'))

            if cs is not None and cd is not None:
                if cs + min(cd, cost_di_v) < cost_si_v + cost_di_v:
                    d_temp_v += cs + min(cd, cost_di_v)
                    T_temp_v.add((si, cs))
                    T_temp_v.add((di, min(cd, cost_di_v)))
                else:
                    d_temp_v += cost_si_v + cost_di_v
                    T_temp_v.add((si, cost_si_v))
                    T_temp_v.add((di, cost_di_v))
            else:
                # クエリ点がまだ T[u] に存在しない場合の処理
                d_temp_v += cost_si_v + cost_di_v
                T_temp_v.add((si, cost_si_v))
                T_temp_v.add((di, cost_di_v))

        if d_temp_v < d[v]:
            d[v] = d_temp_v
            pie[v] = u
            T[v] = T_temp_v
            in_pq = False
            for cost, node in PQ:
                if node == v:
                    in_pq = True
                    break
            if in_pq:
                # 優先度キュー内のキーを減少 (再ヒープ化)
                temp_pq = [(cost, node) for cost, node in PQ if node != v]
                temp_pq.append((d[v], v))
                heapq.heapify(temp_pq)
                PQ[:] = temp_pq
            else:
                heapq.heappush(PQ, (d[v], v))

        return d, pie, T, PQ

    def vehicle_stops(self, st, en, pie, Q, SPC_cache):
        if st not in self.G.nodes or en not in self.G.nodes:
            return []

        vehicle_path_nodes = set()
        current = en
        while current is not None:
            vehicle_path_nodes.add(current)
            if current == st:
                break
            current = pie.get(current)

        if st not in vehicle_path_nodes:
            return [] # 始点から終点への経路が存在しない

        stop_points = {node: False for node in self.G.nodes}

        for si, di in Q:
            if si in self.G.nodes and di in self.G.nodes:
                nearest_si = None
                min_dist_si = float('inf')
                nearest_di = None
                min_dist_di = float('inf')

                for vp_node in vehicle_path_nodes:
                    dist_si_vp = SPC_cache.get((si, vp_node), float('inf'))
                    if dist_si_vp < min_dist_si:
                        min_dist_si = dist_si_vp
                        nearest_si = vp_node
                    elif dist_si_vp == min_dist_si and nearest_si is not None and SPC_cache.get((vp_node, st), float('inf')) < SPC_cache.get((nearest_si, st), float('inf')):
                        nearest_si = vp_node

                    dist_di_vp = SPC_cache.get((di, vp_node), float('inf'))
                    if dist_di_vp < min_dist_di:
                        min_dist_di = dist_di_vp
                        nearest_di = vp_node
                    elif dist_di_vp == min_dist_di and nearest_di is not None and SPC_cache.get((vp_node, st), float('inf')) < SPC_cache.get((nearest_di, st), float('inf')):
                        nearest_di = vp_node

                if nearest_si:
                    stop_points[nearest_si] = True
                if nearest_di:
                    stop_points[nearest_di] = True

        path = [en]
        v = pie.get(en)
        while v != st and v is not None:
            if stop_points.get(v, False):
                path.insert(0, v)
            v = pie.get(v)
        path.insert(0, st)

        return path

    def find_optimal_stops(self, Q, st, en):
        """
        最適な停留所シーケンスを見つけるメイン関数（無向グラフ版）。

        Args:
            Q (list): クエリのリスト [(start_i, end_i), ...]。
            st: 開始停留所。
            en: 終了停留所。

        Returns:
            list: 最適な停留所シーケンス。
        """
        # 4.3, 4.4 無向グラフなので転置は不要。各クエリの始点と終点からの最短経路コストを計算
        SPC_cache = {}
        for si, di in Q:
            if (si, en) not in SPC_cache:
                try:
                    SPC_cache[(si, en)] = nx.dijkstra_path_length(self.G, si, en)
                except nx.NetworkXNoPath:
                    SPC_cache[(si, en)] = float('inf')
            if (di, en) not in SPC_cache:
                try:
                    SPC_cache[(di, en)] = nx.dijkstra_path_length(self.G, di, en)
                except nx.NetworkXNoPath:
                    SPC_cache[(di, en)] = float('inf')
            if (si, st) not in SPC_cache:
                try:
                    SPC_cache[(si, st)] = nx.dijkstra_path_length(self.G, si, st)
                except nx.NetworkXNoPath:
                    SPC_cache[(si, st)] = float('inf')
            if (di, st) not in SPC_cache:
                try:
                    SPC_cache[(di, st)] = nx.dijkstra_path_length(self.G, di, st)
                except nx.NetworkXNoPath:
                    SPC_cache[(di, st)] = float('inf')
            for node in self.G.nodes:
                if (si, node) not in SPC_cache:
                    try:
                        SPC_cache[(si, node)] = nx.dijkstra_path_length(self.G, si, node)
                    except nx.NetworkXNoPath:
                        SPC_cache[(si, node)] = float('inf')
                if (di, node) not in SPC_cache:
                    try:
                        SPC_cache[(di, node)] = nx.dijkstra_path_length(self.G, di, node)
                    except nx.NetworkXNoPath:
                        SPC_cache[(di, node)] = float('inf')
                if (node, st) not in SPC_cache:
                    try:
                        SPC_cache[(node, st)] = nx.dijkstra_path_length(self.G, node, st)
                    except nx.NetworkXNoPath:
                        SPC_cache[(node, st)] = float('inf')


        # 4.5 (d, π, T) ← INIT-VEHICLE(G, Q, st, SPC, SPC_T)
        # SPC_T の代わりに SPC_cache を渡すように変更
        d, pie, T, mark, _ = self.init_vehicle(Q, st, SPC_cache)

        # 4.6 PQ ← G.V
        PQ = [(d[node], node) for node in self.G.nodes]
        heapq.heapify(PQ)
        pq_nodes = set(self.G.nodes)

        # 4.7 while PQ ≠ ∅ do
        while PQ:
            # 4.8 u ← EXTRACT-MIN(PQ)
            current_distance, u = heapq.heappop(PQ)
            if u not in pq_nodes:
                continue
            pq_nodes.remove(u)

            # 4.9 mark(u) ← True
            mark[u] = True

            # 4.10 for each node v ∈ G.Adj[u]
            for v in self.G.neighbors(u):
                # 4.11 if mark(v) = False then
                if not mark[v]:
                    # 4.12 (d, π, T, PQ) ← RELAX-VEHICLE(u, v, w(u, v), st, Q, SPC, SPC_T, d, π, T, PQ)
                    weight = self.G[u][v].get('weight', 1)
                    # SPC_T_cache の代わりに SPC_cache を渡す
                    d, pie, T, PQ = self.relax_vehicle(u, v, weight, st, Q, SPC_cache, d, pie, T, PQ)

        # 4.13 P ← VEHICLE-STOPS(st, en, π, T)
        P = self.vehicle_stops(st, en, pie, Q, SPC_cache)

        # 4.14 return P
        path = []
        length_path = 0
        for i in range(len(P)-1):
            path_str = nx.dijkstra_path(self.G, P[i], P[i+1])
            length_path += len_SP(self.G, P[i], P[i+1], self.len_dic)
            for line in path_str:
                path.append([float(x) for x in line.strip('[]').split(',')])
        position = []
        for line in P:
                position.append([float(x) for x in line.strip('[]').split(',')])
        return path, length_path, position

    def init_vehicle(self, Q, st, SPC_cache):
        d = {}
        pie = {}
        T = {}
        mark = {}
        stop = {}

        for node in self.G.nodes:
            d[node] = float('inf')
            pie[node] = None
            T[node] = set()
            mark[node] = False
            stop[node] = False

        d[st] = 0
        T[st].add((st, 0))

        for q in Q:
            d[st] += SPC_cache.get((q[0], st), 0) + SPC_cache.get((q[1], st), 0)

        for q in Q:
            T[st].add((q[0], SPC_cache.get((q[0], st), 0)))
            T[st].add((q[1], SPC_cache.get((q[1], st), 0)))

        return d, pie, T, mark, stop


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
