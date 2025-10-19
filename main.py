"""
FlaskでサーバをたててWeb上で地図を表示する
"""
from flask import Flask, render_template, request, jsonify
import db, pathSearch
import time
import networkx as nx

app = Flask(__name__)

@app.route("/")
def leafletMap():
    return render_template("index.html")

import time

@app.route('/TSP_path', methods=['POST'])
def TSP_path():
    if request.method == 'POST':
        #リクエストからデータを取得
        data = request.get_json()  
        points = data['points']
        startPoint = data['startPoint']
        endPoint = data['endPoint']
        value = data['value']
        moveDist = float(data['moveDist'])

        #データベースから道路データを取得
        P = points.copy()
        if startPoint:
            P.append(startPoint)
        else:
            startPoint = points[0]
        if endPoint:
            P.append(endPoint)
        else:
            endPoint = points[-1]
        y1, x1, y2, x2 = pathSearch.rectangleArea(P)
        link, length = db.getRectangleRoadData(y1, x1, y2, x2)
        G = pathSearch.linkToGraph(link, length)
        pathSearch.connectGraph(G)
        sp = pathSearch.ShortestPathFinder(G)
        nodes = [sp.nearestNode(p) for p in points]
        startPoint = sp.nearestNode(startPoint)
        endPoint = sp.nearestNode(endPoint)
        # print(points)

        #TSPの場合
        if value == "type1":
            start_time = time.time()
            path, len_path, _ = pathSearch.travelingPath(P, link, length, value)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time})
        
        #パス型TSPの場合
        if value == "type2":
            start_time = time.time()
            path, len_path = pathSearch.path_TSP_zero_edge(startPoint, endPoint, points, link, length)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time})
        
        if value == "type3":
            start_time = time.time()
            path, len_path = pathSearch.path_SHP_full_search(startPoint, endPoint, points, link, length)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time})
        
        if value == "type4":
            start_time = time.time()
            path, len_path = pathSearch.path_SHP_greedy(startPoint, endPoint, points, link, length)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time})
        
        if value == "type5":
            start_time = time.time()
            path, len_path, per, _p = pathSearch.path_SHP_branch_and_bound_with_queue_MST_leaf_speedup(startPoint, endPoint, nodes, sp)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time, 'percent': per})
        
        if value == "type6":
            start_time = time.time()
            path, len_path, per, _p = pathSearch.path_SHP_branch_and_bound_with_queue_MST(startPoint, endPoint, nodes, sp)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time, 'percent': per})
        
        #ORISの場合
        if value == "ORIS":
            #クエリ生成(all destination = en)
            query = []
            for p in nodes:
                query.append((p, endPoint))
            R = pathSearch.Routing(sp)
            start_time = time.time()
            path, len_, position = R.find_optimal_stops(query, startPoint, endPoint, moveDist)
            elapsed_time = time.time() - start_time

            return jsonify({'path': path, 'len': len_, 'position': position, 'exec_time_sec': elapsed_time})

    else:
        return jsonify({'message': 'Invalid request method'})
    
@app.route('/SRP_path', methods=['POST'])
def SRP_path():
    if request.method == 'POST':
        #リクエストからデータを取得
        data = request.get_json()  
        points = data['points']
        startPoint = data['startPoint']
        endPoint = data['endPoint']
        moveDist = float(data['moveDist'])
        value = data['value']

        P = points.copy()
        if startPoint:
            P.append(startPoint)
        else:
            startPoint = points.pop(0)
        if endPoint:
            P.append(endPoint)
        else:
            endPoint = points.pop(-1)
        y1, x1, y2, x2 = pathSearch.rectangleArea(P)
        link, length = db.getRectangleRoadData(y1, x1, y2, x2, 1.05)
        G = pathSearch.linkToGraph(link, length)
        pathSearch.connectGraph(G)
        sp = pathSearch.ShortestPathFinder(G)
        nodes = [sp.nearestNode(p) for p in points]
        startPoint = sp.nearestNode(startPoint)
        endPoint = sp.nearestNode(endPoint)

        #経路探索
        start = time.time()
        if value == "type4":
            path, len, positions_SRP, path_positions, len_walk = pathSearch.BusRouting(startPoint, endPoint, nodes, sp, moveDist)
        if value == "type2":
            path, len, positions_SRP, path_positions, len_walk = pathSearch.new_BusRouting(startPoint, endPoint, nodes, sp, moveDist)


        end = time.time() - start

        return jsonify({'path': path, 'len': len, 'positions_SRP': positions_SRP, 'path_positions': path_positions, 'len_walk': len_walk, 'time': end})
    
    else:
        return jsonify({'message': 'Invalid request method'})
    
@app.route('/test', methods=['POST'])
def test():
    if request.method == 'POST':
        #リクエストからデータを取得
        data = request.get_json()  
        points = data['points']
        moveDist = float(data['moveDist'])

        y1, x1, y2, x2 = pathSearch.rectangleArea(points)
        link, length = db.getRectangleRoadData(y1, x1, y2, x2, 1.05)

        G = pathSearch.linkToGraph(link, length)
        pathSearch.connectGraph(G)
        sp = pathSearch.ShortestPathFinder(G)

        nodes = [sp.nearestNode(p) for p in points]

        start_time = time.time()
        p, _, _ = pathSearch.set_cover(nodes, moveDist, sp)
        elapsed_time = time.time() - start_time

        lst = [ [float(x) for x in s.strip('[]').split(',')] for s in p ]

        return jsonify({'position': lst, 'exec_time_sec': elapsed_time})
    
    else:
        return jsonify({'message': 'Invalid request method'})

if __name__ == "__main__":
    app.run(host="133.68.17.14",port=80,threaded=True)

