"""
経路探索などを行う
"""
import math
import sqlite3
import networkx as nx
import pyproj
import db
import heapq
import itertools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.spatial import KDTree
from pyproj import Transformer
import ast
import pulp
from itertools import combinations
import time

class ShortestPathFinder:
    len_dic = {}
    kdtree = None
    kdtree_nodes = None
    kdtree_coords_km = None
    transformer = None
    zone = None

    def __init__(self, G, use_db_cache=False, db_path="paths.db", weight="weight", bulk_size=100, utm_zone=54):
        connectGraph(G)
        self.G = G
        self.weight = weight
        self.use_db_cache = use_db_cache
        self.bulk_size = bulk_size
        self._bulk_buffer = []
        self.zone = utm_zone

        # UTM zone(日本ゾーン 51-56)、EPSG:3097 + (zone-51)
        epsg_code = 3097 + (utm_zone - 51)
        crs_utm = f"EPSG:{epsg_code}"

        # transformer（zoneが変わったら再作成）
        if (type(self).transformer is None) or (type(self).zone != utm_zone):
            type(self).transformer = Transformer.from_crs("epsg:4326", crs_utm, always_xy=True)
            type(self).zone = utm_zone

        # KDTree初期化（グラフやzone更新時のみ再作成）
        nodes = list(G.nodes)
        if (type(self).kdtree is None or
            type(self).kdtree_nodes != nodes or
            type(self).zone != utm_zone):

            coords_km = []
            for node_str in nodes:
                latlon = ast.literal_eval(node_str)  # [lat, lon] にパース
                # transform(lon, lat)
                x, y = type(self).transformer.transform(latlon[1], latlon[0])
                coords_km.append([x/1000, y/1000])
            type(self).kdtree = KDTree(coords_km)
            type(self).kdtree_nodes = nodes  # = list of str
            type(self).kdtree_coords_km = coords_km

        # DBキャッシュ
        if self.use_db_cache:
            self.conn = sqlite3.connect(db_path)
            self._init_db()
        else:
            self.conn = None

    def _init_db(self):
        cur = self.conn.cursor()
        cur.execute('''
            CREATE TABLE IF NOT EXISTS paths (
                coord1 TEXT,
                coord2 TEXT,
                length REAL,
                PRIMARY KEY (coord1, coord2)
            )
        ''')
        cur.execute('CREATE INDEX IF NOT EXISTS idx_coord1 ON paths(coord1)')
        cur.execute('CREATE INDEX IF NOT EXISTS idx_pair ON paths(coord1, coord2)')
        self.conn.commit()

    def _latlon_to_km(self, latlon):
        # 入力: [lat, lon]リスト
        x, y = type(self).transformer.transform(latlon[1], latlon[0])
        return [x/1000, y/1000]

    def nearestNode(self, p):
        """
        p: [lat, lon]リスト。KDTreeで最近傍ノード（ノードのstr型＝"[lat, lon]"）を返す。
        """
        coord_km = self._latlon_to_km(p)
        dist, idx = type(self).kdtree.query(coord_km)
        return type(self).kdtree_nodes[idx]  # 返値はstr型

    def nodes_within_radius(self, p, r_km):
        """
        p: "[lat, lon]"形式str, r_km: 距離km
        返値: "[lat, lon]" 形式strのリスト
        """
        latlon = ast.literal_eval(p)  # "[lat,lon]" → [lat, lon]リスト
        coord_km = self._latlon_to_km(latlon)
        idxs = type(self).kdtree.query_ball_point(coord_km, r_km)
        return [type(self).kdtree_nodes[i] for i in idxs]
    
    def SP(self, st, en):
        return nx.dijkstra_path(self.G, st, en)

    def len_SP(self, node1, node2):
        # メモリキャッシュ
        try:
            return type(self).len_dic[node1][node2]
        except KeyError:
            pass

        # DBキャッシュ
        if self.use_db_cache:
            cur = self.conn.cursor()
            cur.execute(
                "SELECT length FROM paths WHERE coord1=? AND coord2=?",
                (str(node1), str(node2))
            )
            hit = cur.fetchone()
            if hit is not None:
                length = hit[0]
                if node1 not in self.len_dic:
                    type(self).len_dic[node1] = {}
                if length is not None:
                    type(self).len_dic[node1][node2] = length
                    # 逆方向も保存
                    if node2 not in self.len_dic:
                        type(self).len_dic[node2] = {}
                    type(self).len_dic[node2][node1] = length
                    return length
                else:
                    return None

        # 計算
        dist = self._dijkstra(node1, node2)
        if node1 not in self.len_dic:
            type(self).len_dic[node1] = {}
        if node2 not in self.len_dic:
            type(self).len_dic[node2] = {}
        type(self).len_dic[node1][node2] = dist
        type(self).len_dic[node2][node1] = dist  # ← 逆方向も格納

        # DBへのバッファ
        if self.use_db_cache:
            self._bulk_buffer.append((str(node1), str(node2), dist))
            self._bulk_buffer.append((str(node2), str(node1), dist))  # DBも逆方向追記
            if len(self._bulk_buffer) >= self.bulk_size:
                self._flush_bulk()
        
        return dist

    def _dijkstra(self, start, goal):
        try:
            return nx.shortest_path_length(self.G, source=start, target=goal, weight=self.weight)
        except nx.NetworkXNoPath:
            print("SP error")
            return None

    def _flush_bulk(self):
        if self.use_db_cache and self._bulk_buffer:
            cur = self.conn.cursor()
            cur.executemany(
                "INSERT OR IGNORE INTO paths (coord1, coord2, length) VALUES (?, ?, ?)",
                self._bulk_buffer
            )
            self.conn.commit()
            self._bulk_buffer = []

    def close(self):
        self._flush_bulk()
        if self.conn:
            self.conn.close()

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
#半径rの円を囲む正方形の左上と右下の点 r:km
def aroundRectagleArea(y, x, r):
    r = r * 1000
    d = r * math.sqrt(6) / 2 #内接正方形の1.5倍の面積
    grs80 = pyproj.Geod(ellps='GRS80')
    x1, y1, _back1 = grs80.fwd(float(y), float(x), 135, d)
    x2, y2, _back2 = grs80.fwd(float(y), float(x), 315, d)
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
                G.add_weighted_edges_from([(node1, node2, 1e9)])

#リンクからグラフ生成
def linkToGraph(link, length):
    edges = []
    for i in range(len(link)):
        edges.append((str(link[i][0]), str(link[i][1]), length[i]))
    G = nx.Graph()
    G.add_weighted_edges_from(edges)
    return G
    
#巡回経路
def travelingPath(points, link, length, value):
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
    sp = ShortestPathFinder(G_temp)
    for u in positions:
        for v in positions:
            if positions.index(u) < positions.index(v):
                G.add_edge(u, v, weight=sp.len_SP(u, v))
    
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
        length += sp.len_SP(tsp[i], tsp[i+1])
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    return path, length, tsp

#経由点決定(あまのさん)
def viterbi_ver1(tsp, candidates, len_dic, G):
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
        if i == len(candidates)-1:
            n = 0
        for node in candidates[n]: 
            dist_min = float('inf')
            node_min = ""
            for node_prev in candidates[i]:
                dist = len_SP(G, node, node_prev, len_dic) + path_length[node_prev]
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

    return positions_SRP

#経由点決定(SHP)
def viterbi_ver2(shp, candidates, sp, alpha=0.01):
    positions_SRP = []
    path_length = {shp[0]: 0}
    path_backtrack = {}

    for i in range(1, len(candidates)):  # 1始まり
        for node in candidates[i]:
            dist_min = float('inf')
            node_min = None
            for node_prev in candidates[i-1]:
                a = sp.len_SP(node, node_prev)
                b = path_length[node_prev]
                c = sp.len_SP(node, shp[i]) * alpha
                dist = a + b + c
                if dist < dist_min:
                    dist_min = dist
                    node_min = node_prev
            if node_min is None:
                raise RuntimeError(f'No predecessor found for node {node} at step {i}')
            path_length[node] = dist_min
            if node in path_backtrack:
                path_backtrack[node][i] = node_min
            else:
                path_backtrack[node] = {i: node_min}

    # 終点から逆順で戻す
    node = shp[-1]  # 終点
    for i in reversed(range(1, len(candidates))):
        positions_SRP.insert(0, node)
        node = path_backtrack[node][i]
    positions_SRP.insert(0, node)  # 始点

    if len(positions_SRP) != len(shp):
        raise RuntimeError(f'positions_SRPとshpの長さが不一致です: {len(positions_SRP)} != {len(shp)}')

    points_move_dic = {}
    for i in range(len(shp)):
        if positions_SRP[i] in points_move_dic:
            points_move_dic[positions_SRP[i]].append(shp[i])
        else:
            points_move_dic[positions_SRP[i]] = [shp[i]]
    return positions_SRP, points_move_dic

#2-opt
def two_opt(path, sp):
    size = len(path)
    while True:
        count = 0
        for i in range(size - 2):
            i1 = i + 1
            for j in range(i + 2, size):
                # 通常通り範囲内のj1だけ
                j1 = j + 1
                if j1 >= size:
                    continue  # パスの端点はその先がないので除外
                # 始点・終点を固定したまま内部を入れ替える
                l1 = sp.len_SP(path[i], path[i1])
                l2 = sp.len_SP(path[j], path[j1])
                l3 = sp.len_SP(path[i], path[j])
                l4 = sp.len_SP(path[i1], path[j1])
                if l1 + l2 > l3 + l4:
                    # つなぎかえる
                    new_path = path[i1:j+1]
                    path[i1:j+1] = new_path[::-1]
                    count += 1
        if count == 0:
            break
    return path

def bulk_prewarm_len_SP(sp, node_lists):
    """
    sp.len_SPを高速化するため、node_lists（2重リスト）中の現れ得る全ペアについて一括してキャッシュに登録する
    """
    all_nodes = set()
    for nds in node_lists:
        all_nodes.update(nds)
    all_nodes = list(all_nodes)
    for i in range(len(all_nodes)):
        for j in range(len(all_nodes)):
            _ = sp.len_SP(all_nodes[i], all_nodes[j])  # 不要な場合は片側・自己ループ除外も可

def greedy_set_cover(universe, subsets):
    """
    貪欲法による集合被覆問題の実装
    universe: カバー対象の要素集合（set型）
    subsets: 部分集合のリスト。各要素は set型。
    
    返値: 被覆に使った集合のインデックスリスト、被覆のリスト
    """
    covered = set()           # すでにカバーした要素
    cover_indices = []        # 採用した部分集合のインデックス
    cover_sets = []           # それぞれがカバーした集合
    
    subsets = [s.copy() for s in subsets] # 安全のためコピー

    while covered != universe:
        # 求めるのは「未カバー要素を最も多く含む部分集合」
        max_new_cover = set()
        max_idx = -1
        for idx, s in enumerate(subsets):
            new_cover = s - covered           # この部分集合が今回新たにカバーする要素
            if len(new_cover) > len(max_new_cover):
                max_new_cover = new_cover
                max_idx = idx
        
        # これ以上増えない＝カバーできない要素がある
        if not max_new_cover:
            print("Error: カバー不可能な要素があります")
            break

        # 見つかった代表集合を記録
        cover_indices.append(max_idx)
        cover_sets.append(subsets[max_idx])
        covered |= max_new_cover   # coveredを更新

    return cover_indices, cover_sets

#集合被覆問題
def set_cover(nodes, moveDist, sp):
    S = [set(sp.nodes_within_radius(n, moveDist)) for n in nodes] #各ノードの周辺ノード
    
    S_int_idx = []
    # 単一の集合
    for idx in range(len(S)):
        S_int_idx.append({idx})
    # 2以上全てのサイズについて組み合わせを探索
    for k in range(2, len(S)+1):
        for idxs in combinations(range(len(S)), k):
            # ノード同士が遠ければスキップ
            skip = False
            for idx1, idx2 in combinations(idxs, 2):
                if sp.len_SP(nodes[idx1], nodes[idx2]) > moveDist*2.5:
                    skip = True
                    break
            if skip:
                continue

            # 共通部分を求める
            intersect = S[idxs[0]].copy()
            for idx in idxs[1:]:
                intersect &= S[idx]
            if intersect:
                S_int_idx.append(set(idxs))

    cover_indices, cover_sets = greedy_set_cover(set(range(len(S))), S_int_idx)
    print(cover_sets)
    points = []
    points_set = []
    uni = set.union(*S)
    for s in cover_sets:
            if len(s) == 1:
                points.extend(list(S[next(iter(s))]))
                points_set.append(list(S[next(iter(s))]))
            else:
                s_temp = uni
                for idx in s:
                    s_temp = s_temp & S[idx]
                points.extend(list(s_temp))
                points_set.append(list(s_temp))
    
    return points, points_set, cover_sets

def new_BusRouting(st, en, points, sp, moveDist):
    import time
    
    time0 = time.time()

    # set coverによりバス停を決定
    t1 = time.time()
    _n, nodes_set, cover_sets = set_cover(points, moveDist, sp)
    t2 = time.time()

    nodes = []
    for ns in nodes_set:
        nodes.append(ns[0])

    # SHPを解く
    t3 = time.time()
    _a, _b, _c, shp = path_SHP_branch_and_bound_with_queue_MST(st, en, nodes, sp)
    t4 = time.time()

    # 対応付け
    t5 = time.time()
    candidates = []
    candidates.append([shp[0]])
    temp_path = shp[1:-1]
    index_map = {a: i for i, a in enumerate(nodes)}
    candidates.extend([nodes_set[index_map[c]] for c in temp_path])
    candidates.append([shp[-1]])
    t6 = time.time()

    # Viterbi
    t_vit1 = time.time()
    positions_SRP, points_move_dic = viterbi_ver2(shp, candidates, sp)
    t_vit2 = time.time()

    # two_opt
    t_opt1 = time.time()
    # positions_SRP = two_opt(positions_SRP, sp)
    t_opt2 = time.time()

    # 巡回順に最短経路を求めて返却
    t9 = time.time()
    path = []
    length_SRP = 0
    for i in range(len(positions_SRP)-1):
        path_str = sp.SP(positions_SRP[i], positions_SRP[i+1])
        length_SRP += sp.len_SP(positions_SRP[i], positions_SRP[i+1])
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    t10 = time.time()

    # 各ポイント→割り当てバス停までの経路
    t11 = time.time()
    path_positions = []
    positions_SRP_a = []
    len_walk = 0
    for i in range(len(positions_SRP)):
        positions_SRP_a.append(positions_SRP[i].strip("[]").split(","))

    # ここから修正版
    for i, p in enumerate(positions_SRP[1:-1]):
        assigned_node = points_move_dic[p][0]
        idx = nodes.index(assigned_node)
        for idx2 in cover_sets[idx]:
            path_str = sp.SP(points[idx2], p)
            len_walk += sp.len_SP(points[idx2], p)
            path_temp = []
            for line in path_str:
                path_temp.append([float(x) for x in line.strip('[]').split(',')])
            path_positions.append(path_temp)
    # ここまで修正版

    t12 = time.time()

    print("--- 実行時間計測 ---")
    print("set_cover: {:.3f}秒".format(t2-t1))
    print("SHP: {:.3f}秒".format(t4-t3))
    print("対応付け: {:.3f}秒".format(t6-t5))

    print("Viterbi: {:.3f}秒".format(t_vit2-t_vit1))
    print("two_opt: {:.3f}秒".format(t_opt2-t_opt1))

    print("巡回経路計算: {:.3f}秒".format(t10-t9))
    print("移動先までの経路: {:.3f}秒".format(t12-t11))
    print("全体: {:.3f}秒".format(time.time()-time0))
    print("-------------------")

    return path, length_SRP, positions_SRP_a, path_positions, len_walk
#バス停問題
def BusRouting(st, en, points, sp, moveDist):
    #SHPを解く
    _a, _b, _c, shp = path_SHP_branch_and_bound_with_queue_MST(st, en, points, sp)

    #乗客の移動候補ノードを取得
    candidates = []
    candidates.append([shp[0]])
    temp_path = shp[1:-1]
    for p in temp_path:
        candidates.append(sp.nodes_within_radius(p, moveDist))
    candidates.append([shp[-1]])

    print("SHP len : "+str(len(shp)))

    #順に候補点から経由点を決定
    positions_SRP, points_move_dic = viterbi_ver2(shp, candidates, sp)
    positions_SRP = two_opt(positions_SRP, sp)

    
    #巡回順に最短経路を求めて返却
    path = []
    length_SRP = 0
    for i in range(len(positions_SRP)-1):
        path_str = sp.SP(positions_SRP[i], positions_SRP[i+1])
        length_SRP += sp.len_SP(positions_SRP[i], positions_SRP[i+1])
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])

    #各移動先までの経路
    path_positions = []
    positions_SRP_a = []
    len_walk = 0
    for i in range(len(positions_SRP)):
        positions_SRP_a.append(positions_SRP[i].strip("[]").split(","))
        point = points_move_dic[positions_SRP[i]][0]
        if point != positions_SRP[i]:
            path_str = sp.SP(point, positions_SRP[i])
            len_walk += sp.len_SP(point, positions_SRP[i])
            path_temp = []
            for line in path_str:
                path_temp.append([float(x) for x in line.strip('[]').split(',')])
            path_positions.append(path_temp)

    return path, length_SRP, positions_SRP_a, path_positions, len_walk

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

def path_TSP_zero_edge(st, en, points, link, length):
    # 都市ノードリストの作成（st, en, points全部）
    positions = [str(nearestNode(p, link)) for p in points]
    st = str(nearestNode(st, link))
    en = str(nearestNode(en, link))
    
    # st, en, pointsすべてを集合化して重複排除
    node_list = list(set(positions + [st, en]))
    
    # 巡回セールスマン問題用の完全グラフを作る
    G = nx.Graph()
    for i, n in enumerate(node_list):
        G.add_node(n, index=i)
    
    # 都市間最短経路探索用グラフ
    G_temp = linkToGraph(link, length)
    connectGraph(G_temp)
    
    # 全点間完全グラフを構築する
    sp = ShortestPathFinder(G_temp)
    for i in range(len(node_list)):
        for j in range(i+1, len(node_list)):
            u, v = node_list[i], node_list[j]
            if (u == st and v == en) or (u == en and v == st):
                continue  # st-en間だけはあとで追加（ダミーエッジ）
            G.add_edge(u, v, weight=sp.len_SP(u, v))
    G.add_edge(st, en, weight=0.00001)  # st-en間ダミーエッジ

    # 完全グラフになっているか確認
    assert nx.is_connected(G), "G is not connected!"

    # 巡回セールスマン問題を近似解で解く（ダミーエッジを活かす）
    tsp = list(nx.algorithms.approximation.simulated_annealing_tsp(G, init_cycle="greedy"))
    tsp.pop()  # 巡回路から最終ノード=始点の余分を消す

    # スタート・ゴールが最初と最後になるよう並び替え
    tsp = reorder_tsp_st_to_en(tsp, st, en)

    # 巡回順に最短経路を求めて返す
    path = []
    total_length = 0
    for i in range(len(tsp)-1):
        path_str = nx.dijkstra_path(G_temp, tsp[i], tsp[i+1])
        total_length += sp.len_SP(tsp[i], tsp[i+1])
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    return path, total_length

def path_SHP_full_search(st, en, points, link, length):
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
    sp = ShortestPathFinder(G_temp)

    # 都市間の最短経路によるグラフ構築
    all_points = positions + [st, en]
    for u, v in itertools.combinations(all_points, 2):
        if (u == st and v == en) or (u == en and v == st):
            continue
        G.add_edge(u, v, weight=sp.len_SP(u, v))

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
            length_sum += sp.len_SP(best_order[i], best_order[i+1])
            for line in path_str:
                all_path.append([float(x) for x in line.strip('[]').split(',')])

    return all_path, length_sum

def path_SHP_greedy(st, en, points, link, length):
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
    sp = ShortestPathFinder(G_temp)

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
            distances = [sp.len_SP(current, city) for city in candidates]
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
        length_sum += sp.len_SP(order[i], order[i+1])
        for line in path_str:
            path.append([float(x) for x in line.strip('[]').split(',')])
    return path, length_sum

def path_SHP_branch_and_bound_with_queue(st, en, points, link, length):
    # 都市ノードリスト
    positions = [str(nearestNode(p, link)) for p in points]
    st = str(nearestNode(st, link))
    en = str(nearestNode(en, link))

    # 都市セット（重複排除・順序問わず）
    all_cities = list(set(positions + [st, en]))
    N = len(all_cities)

    # 1. 経路グラフ準備
    #    G_temp上で都市間距離をlen_SPで求める (最短路長)
    G_temp = linkToGraph(link, length)
    connectGraph(G_temp)
    sp = ShortestPathFinder(G_temp)

    # 2. 距離テーブル構築
    #    city_dist[u][v] = len_SP(u,v)
    city_dist = {}
    for u in all_cities:
        city_dist[u] = {}
        for v in all_cities:
            if u == v:
                city_dist[u][v] = 0
            else:
                city_dist[u][v] = sp.len_SP(u, v)

    N = len(all_cities)
    n_middle = N - 2  # st, enを除いた都市数

    # 訪問ノード数カウンタ
    node_count = 0

    # 全探索パス数（始点と終点以外全部積み換えた場合）
    if n_middle <= 0:
        full_search_steps = 1
    else:
        full_search_steps = math.factorial(n_middle)

    # 3. 分枝限定(優先度キュー付き)
    best = {'cost': float('inf'), 'path': None}
    # --- 貪欲法による初期解探索 ---
    curr = st
    visited_greedy = {st}
    greedy_path = [st]
    greedy_cost = 0

    while len(visited_greedy) < N - 1:  # en以外全部
        candidates = set(all_cities) - visited_greedy - {en}
        if not candidates:
            break  # 全て訪問済み
        # 最も近い未訪問都市を選ぶ
        next_city = min(candidates, key=lambda v: city_dist[curr][v])
        greedy_path.append(next_city)
        greedy_cost += city_dist[curr][next_city]
        visited_greedy.add(next_city)
        curr = next_city

    # 最後に終点enへ
    greedy_path.append(en)
    greedy_cost += city_dist[curr][en]

    # 初期解としてセット
    best = {'cost': greedy_cost, 'path': greedy_path}

    def lower_bound(u, visited, curr_cost):
        # 下界見積もり:
        # 1) 現コスト
        # 2) 今いるノード→未訪問都市の最小コスト（なければ0）
        # 3) 各未訪問都市→未訪問・enへの最小
        unvisited = set(all_cities) - visited - {en}
        lb = curr_cost
        # 2)
        if unvisited:
            lb += min(city_dist[u][v] for v in unvisited)
        # 3)
        for v in unvisited:
            min_edge = min([city_dist[v][w] for w in (unvisited | {en}) if w != v])
            lb += min_edge
        return lb

    # キュー：(推定トータル, 現コスト, 現ノード, path, visited_set)
    #          ※推定トータル = curr_cost + 下界見積もり
    heap = []
    heapq.heappush(heap, (0, 0, st, [st], {st}))
    while heap:
        est_total, curr_cost, u, path, visited = heapq.heappop(heap)
        node_count += 1  # ←ここでカウント
        # すべて通っていればenに移動して終了判定
        if len(visited) == N - 1 and en not in visited:
            total = curr_cost + city_dist[u][en]
            if total < best['cost']:
                best['cost'] = total
                best['path'] = path + [en]
            continue
        # 下界限定（暫定より劣化なら以降無視）
        lb = lower_bound(u, visited, curr_cost)
        if lb >= best['cost']:
            continue
        # 分枝（未訪問&en以外）
        for v in (set(all_cities) - visited - {en}):
            new_cost = curr_cost + city_dist[u][v]
            new_path = path + [v]
            new_visited = visited | {v}
            est = lower_bound(v, new_visited, new_cost)
            heapq.heappush(heap, (est, new_cost, v, new_path, new_visited))

    # 結果パスから最短経路復元（経路上の始点/終点でDijkstra、全点実座標列に直す）
    ret_path = []
    path_len = 0
    if best['path']:
        for i in range(len(best['path'])-1):
            spa = nx.dijkstra_path(G_temp, best['path'][i], best['path'][i+1])
            path_len += sp.len_SP(best['path'][i], best['path'][i+1])
            for line in spa:
                ret_path.append([float(x) for x in line.strip('[]').split(',')])

    percent = (node_count / full_search_steps) * 100
    # print(f"探索ノード数: {node_count} / 全探索: {full_search_steps} ({percent:.2f} %)")
    return ret_path, path_len, percent

def path_SHP_branch_and_bound_with_queue_MST(st, en, points, sp):
    # 都市セット（重複排除・順序問わず）
    all_cities = list(set(points + [st, en]))
    N = len(all_cities)

    # 2. 距離テーブル構築
    #    city_dist[u][v] = len_SP(u,v)
    city_dist = {}
    for u in all_cities:
        city_dist[u] = {}
        for v in all_cities:
            if u == v:
                city_dist[u][v] = 0
            else:
                city_dist[u][v] = sp.len_SP(u, v)

    N = len(all_cities)
    n_middle = N - 2  # st, enを除いた都市数

    # 訪問ノード数カウンタ
    node_count = 0

    # 全探索パス数（始点と終点以外全部積み換えた場合）
    if n_middle <= 0:
        full_search_steps = 1
    else:
        full_search_steps = math.factorial(n_middle)

    # 3. 分枝限定(優先度キュー付き)
    best = {'cost': float('inf'), 'path': None}
    # --- 貪欲法による初期解探索 ---
    curr = st
    visited_greedy = {st}
    greedy_path = [st]
    greedy_cost = 0

    while len(visited_greedy) < N - 1:  # en以外全部
        candidates = set(all_cities) - visited_greedy - {en}
        if not candidates:
            break  # 全て訪問済み
        # 最も近い未訪問都市を選ぶ
        next_city = min(candidates, key=lambda v: city_dist[curr][v])
        greedy_path.append(next_city)
        greedy_cost += city_dist[curr][next_city]
        visited_greedy.add(next_city)
        curr = next_city

    # 最後に終点enへ
    greedy_path.append(en)
    greedy_cost += city_dist[curr][en]

    # 初期解としてセット
    best = {'cost': greedy_cost, 'path': greedy_path}

    def lower_bound(u, visited, curr_cost):
        """
        u: 現在地
        visited: 訪問済セット
        curr_cost: ここまでの累積コスト
        return: この部分経路から最短でも必要となるMST下界コスト
        """
        unvisited = set(all_cities) - visited - {en}
        lb = curr_cost

        if not unvisited:
            return lb

        # (1) unvisitedノード群のみのMSTコスト
        if len(unvisited) >= 2:
            # mst用networkxグラフを都度構築
            H = nx.Graph()
            for a in unvisited:
                for b in unvisited:
                    if a != b:
                        H.add_edge(a, b, weight=city_dist[a][b])
            mst_edges = nx.minimum_spanning_edges(H, data=True)
            mst_cost = sum(e[2]['weight'] for e in mst_edges)
            lb += mst_cost

        # (2) 今いるノード→unvisitedへの最小コスト
        min_enter = min(city_dist[u][v] for v in unvisited)
        lb += min_enter

        # (3) unvisited→終点への最小
        min_exit = min(city_dist[v][en] for v in unvisited)
        lb += min_exit

        return lb

    # キュー：(推定トータル, 現コスト, 現ノード, path, visited_set)
    #          ※推定トータル = curr_cost + 下界見積もり
    heap = []
    heapq.heappush(heap, (0, 0, st, [st], {st}))
    while heap:
        est_total, curr_cost, u, path, visited = heapq.heappop(heap)
        node_count += 1  # ←ここでカウント
        # すべて通っていればenに移動して終了判定
        if len(visited) == N - 1 and en not in visited:
            total = curr_cost + city_dist[u][en]
            if total < best['cost']:
                best['cost'] = total
                best['path'] = path + [en]
            continue
        # 下界限定（暫定より劣化なら以降無視）
        lb = lower_bound(u, visited, curr_cost)
        if lb >= best['cost']:
            continue
        # 分枝（未訪問&en以外）
        for v in (set(all_cities) - visited - {en}):
            new_cost = curr_cost + city_dist[u][v]
            new_path = path + [v]
            new_visited = visited | {v}
            est = lower_bound(v, new_visited, new_cost)
            heapq.heappush(heap, (est, new_cost, v, new_path, new_visited))

    # 結果パスから最短経路復元（経路上の始点/終点でDijkstra、全点実座標列に直す）
    ret_path = []
    path_len = 0
    if best['path']:
        for i in range(len(best['path'])-1):
            spa = sp.SP(best['path'][i], best['path'][i+1])
            path_len += sp.len_SP(best['path'][i], best['path'][i+1])
            for line in spa:
                ret_path.append([float(x) for x in line.strip('[]').split(',')])
    
    percent = (node_count / full_search_steps) * 100
    return ret_path, path_len, percent, best['path']

def path_SHP_branch_and_bound_with_queue_MST_leaf_speedup(st, en, points, sp):
    """
    B&B探索において、MSTの下界見積もりを「葉ノード削除時のみ差分」でキャッシュ再利用して高速化する。
    :param st: 始点
    :param en: 終点
    :param points: 途中必須地点のリスト
    :param sp: 最短経路長/列計算オブジェクト (sp.len_SP(u, v), sp.SP(u, v)が使える)
    :return: ret_path, path_len, percent, best['path']
    """

    # Step 1. 都市リスト生成
    all_cities = list(set(points + [st, en]))
    N = len(all_cities)

    # Step 2. 距離テーブル構築
    city_dist = {}
    for u in all_cities:
        city_dist[u] = {}
        for v in all_cities:
            if u == v:
                city_dist[u][v] = 0
            else:
                city_dist[u][v] = sp.len_SP(u, v)

    n_middle = N - 2  # st, enを除いた途中都市数

    # 探索ノード数カウンタ
    node_count = 0

    # 全探索パス数 (始点/終点以外すべてを巡る順列)
    full_search_steps = 1 if n_middle <= 0 else math.factorial(n_middle)

    # 初期解: 貪欲法+MSTによる枝刈り高速化
    best = {'cost': float('inf'), 'path': None}
    curr = st
    visited_greedy = {st}
    greedy_path = [st]
    greedy_cost = 0
    while len(visited_greedy) < N - 1:  # en以外全部
        candidates = set(all_cities) - visited_greedy - {en}
        if not candidates:
            break
        next_city = min(candidates, key=lambda v: city_dist[curr][v])
        greedy_path.append(next_city)
        greedy_cost += city_dist[curr][next_city]
        visited_greedy.add(next_city)
        curr = next_city
    greedy_path.append(en)
    greedy_cost += city_dist[curr][en]
    best = {'cost': greedy_cost, 'path': greedy_path}

    # --- MSTキャッシュ(部分再利用) ---
    mst_cache = {}  # frozenset(unvisitedノード): (mst_cost, mst_edges)

    def get_MST_cost_and_edges(unvisited_cities):
        key = frozenset(unvisited_cities)
        if key in mst_cache:
            return mst_cache[key]
        if len(unvisited_cities) < 2:
            mst_cache[key] = (0, [])
            return 0, []
        H = nx.Graph()
        for a in unvisited_cities:
            for b in unvisited_cities:
                if a != b:
                    H.add_edge(a, b, weight=city_dist[a][b])
        mst_edges = list(nx.minimum_spanning_edges(H, data=True))
        cost = sum(e[2]['weight'] for e in mst_edges)
        edges = [(e[0], e[1], e[2]['weight']) for e in mst_edges]
        mst_cache[key] = (cost, edges)
        return cost, edges

    def lower_bound(u, visited, curr_cost, prev_unvisited=None, prev_mst_cost=None, prev_mst_edges=None, removed_node=None):
        """
        現在地u, 訪問済set, 累積コスト + 直前MST情報による差分高速化
        prev_unvisited, ...: 1つ前のunvisited等
        removed_node: 今回追加(=前回unvisitedから外れた点, 候補v)
        """
        unvisited = set(all_cities) - visited - {en}
        lb = curr_cost

        mst_cost, mst_edges = None, None
        # 差分最適化: 前回MSTから葉ノード1点だけ削除した場合
        if not unvisited:
            mst_cost = 0
            mst_edges = []
        elif prev_unvisited is not None and prev_mst_cost is not None and prev_mst_edges is not None and removed_node is not None:
            if unvisited == prev_unvisited - {removed_node}:
                # removed_nodeが前回MSTで葉ノードか
                deg = 0
                last_edge = None
                for e in prev_mst_edges:
                    if e[0] == removed_node or e[1] == removed_node:
                        deg += 1
                        last_edge = e
                if deg == 1:
                    mst_cost = prev_mst_cost - last_edge[2]
                    mst_edges = [e for e in prev_mst_edges if not (removed_node in e[:2])]
        # 差分適用できない場合はフル計算
        if mst_cost is None:
            mst_cost, mst_edges = get_MST_cost_and_edges(unvisited)
        lb += mst_cost

        # (2) u→unvisited最小
        if unvisited:
            min_enter = min(city_dist[u][v] for v in unvisited)
            lb += min_enter
            min_exit = min(city_dist[v][en] for v in unvisited)
            lb += min_exit
        return lb, unvisited, mst_cost, mst_edges  # 下界と次のMST情報

    # ヒープ: (推定total下界, 現コスト, 現ノード, path, visited_set, prev_unvisited, prev_mst_cost, prev_mst_edges, last_addedノード)
    heap = []
    lb0, unvisited0, mst_cost0, mst_edges0 = lower_bound(st, {st}, 0, None, None, None, None)
    heapq.heappush(heap, (lb0, 0, st, [st], {st}, unvisited0, mst_cost0, mst_edges0, None))

    while heap:
        est_total, curr_cost, u, path, visited, prev_unvisited, prev_mst_cost, prev_mst_edges, last_added = heapq.heappop(heap)
        node_count += 1
        # すべて通っていればenに移動して終了判定
        if len(visited) == N - 1 and en not in visited:
            total = curr_cost + city_dist[u][en]
            if total < best['cost']:
                best['cost'] = total
                best['path'] = path + [en]
            continue
        # 分枝
        for v in (set(all_cities) - visited - {en}):
            new_cost = curr_cost + city_dist[u][v]
            new_path = path + [v]
            new_visited = visited | {v}
            # 下界計算: prev_unvisited, prev_mst_cost, ...を渡し、今回はvを取り除く
            lb, new_unvisited, new_mst_cost, new_mst_edges = lower_bound(
                v, new_visited, new_cost,
                prev_unvisited, prev_mst_cost, prev_mst_edges, v
            )
            if lb >= best['cost']:
                continue
            heapq.heappush(heap, (
                lb, new_cost, v, new_path, new_visited,
                new_unvisited, new_mst_cost, new_mst_edges, v
            ))

    # パス最短経路復元
    ret_path = []
    path_len = 0
    if best['path']:
        for i in range(len(best['path'])-1):
            spa = sp.SP(best['path'][i], best['path'][i+1])
            path_len += sp.len_SP(best['path'][i], best['path'][i+1])
            for line in spa:
                ret_path.append([float(x) for x in line.strip('[]').split(',')])

    percent = (node_count / full_search_steps) * 100
    return ret_path, path_len, percent, best['path']
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
        sp = ShortestPathFinder(self.G)
        path = []
        length_path = 0
        for i in range(len(P)-1):
            path_str = nx.dijkstra_path(self.G, P[i], P[i+1])
            length_path += sp.len_SP(P[i], P[i+1])
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
    sp = ShortestPathFinder(G)
    positions = []
    for p in points:
        positions.append(str(nearestNode(p, link)))

    data = {}
    DM = []
    for p in positions:
        d = []
        for q in positions:
            d.append(sp.len_SP(p, q))
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
