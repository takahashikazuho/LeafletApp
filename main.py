"""
FlaskでサーバをたててWeb上で地図を表示する
"""
from flask import Flask, render_template, request, jsonify
import db, pathSearch
import threading
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
            path, len_path = pathSearch.path_TSP_full_search(startPoint, endPoint, points, link, length)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time})
        
        if value == "type4":
            start_time = time.time()
            path, len_path = pathSearch.path_TSP_greedy(startPoint, endPoint, points, link, length)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time})
        
        if value == "type5":
            start_time = time.time()
            path, len_path, per = pathSearch.path_TSP_branch_and_bound_with_queue(startPoint, endPoint, points, link, length)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time, 'percent': per})
        
        if value == "type6":
            start_time = time.time()
            path, len_path, per = pathSearch.path_TSP_branch_and_bound_with_queue_MST(startPoint, endPoint, points, link, length)
            elapsed_time = time.time() - start_time
            return jsonify({'path': path, 'len': len_path, 'exec_time_sec': elapsed_time, 'percent': per})
        
        #ORISの場合
        if value == "ORIS":
            p_temp = []
            for p in points:
                p_temp.append(pathSearch.nearestNode(p, link))
            points = p_temp

            #pointsの先頭から二個組ずつをクエリとする
            query = []
            for i in range(0, len(points) - 1, 2):
                query.append((str(points[i]), str(points[i + 1])))

            R = pathSearch.Routing(link, length)
            start_time = time.time()
            path, len_, position = R.find_optimal_stops(query, str(pathSearch.nearestNode(startPoint, link)), str(pathSearch.nearestNode(endPoint, link)))
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

        if startPoint:
            points.append(startPoint)
        if endPoint:
            points.append(endPoint)

        #データベースから道路データを取得
        y1, x1, y2, x2 = pathSearch.rectangleArea(points)
        link, length = db.getRectangleRoadData(y1, x1, y2, x2)

        #経路探索
        start = time.perf_counter()
        path, len, points_SRP, positions_SRP, path_positions, len_walk = pathSearch.sharedRidePath(points, link, length, moveDist, value)
        end = time.perf_counter()
        print(end - start)

        return jsonify({'path': path, 'len': len, 'points_SRP': points_SRP, 'positions_SRP': positions_SRP, 'path_positions': path_positions, 'len_walk': len_walk})
    
    else:
        return jsonify({'message': 'Invalid request method'})

if __name__ == "__main__":
    app.run(host="133.68.17.14",port=80,threaded=True)

