"""
FlaskでサーバをたててWeb上で地図を表示する
"""
from flask import Flask, render_template, request, jsonify
import db, pathSearch
import threading
import time
import networkx as nx

app = Flask(__name__)

len_dic = {}
def load_data():
    global len_dic
    points = [[35.18441016255083, 136.9190883636475], [35.15573127979675, 136.97341918945315]]
    y1, x1, y2, x2 = pathSearch.rectangleArea(points)
    link, length = db.getRectangleRoadData(y1, x1, y2, x2)
    G = pathSearch.linkToGraph(link, length)
    len_dic = dict(nx.all_pairs_dijkstra_path_length(G))
    print("---data loaded---")

@app.route("/")
def leafletMap():
    return render_template("index.html")

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
        P = points
        if startPoint:
            P.append(startPoint)
        if endPoint:
            P.append(endPoint)
        y1, x1, y2, x2 = pathSearch.rectangleArea(P)
        link, length = db.getRectangleRoadData(y1, x1, y2, x2)

        #TSPの場合
        if value == "type1":
            if startPoint:
                points.append(startPoint)
            if endPoint:
                points.append(endPoint)
            path, len_path, _ = pathSearch.travelingPath(points, link, length, value, len_dic)
            return jsonify({'path': path, 'len': len_path})
        
        #パス型TSPの場合
        if value == "type2":
            path, len_path = pathSearch.path_TSP(startPoint, endPoint, points, link, length, len_dic)
            return jsonify({'path': path, 'len': len_path})
        
        #ORISの場合
        if value == "type3":
            p_temp = []
            for p in points:
                p_temp.append(pathSearch.nearestNode(p, link))
            points = p_temp

            #pointsの先頭から二個組ずつをクエリとする
            query = []
            for i in range(0, len(points) - 1, 2):
                query.append((str(points[i]), str(points[i + 1])))

            R = pathSearch.Routing(link, length)
            path, len_, position = R.find_optimal_stops(query, str(pathSearch.nearestNode(startPoint, link)), str(pathSearch.nearestNode(endPoint, link)))

            return jsonify({'path': path, 'len': len_, 'position': position})
        
    
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
        link, length = db.getRectangleRoadData(y1, x1, y2, x2, 1.25)

        #経路探索
        start = time.perf_counter()
        path, len, points_SRP, positions_SRP, path_positions, len_walk = pathSearch.sharedRidePath(points, link, length, moveDist, value, len_dic)
        end = time.perf_counter()
        print(end - start)

        return jsonify({'path': path, 'len': len, 'points_SRP': points_SRP, 'positions_SRP': positions_SRP, 'path_positions': path_positions, 'len_walk': len_walk})
    
    else:
        return jsonify({'message': 'Invalid request method'})

if __name__ == "__main__":
    data_loading_thread = threading.Thread(target=load_data)
    data_loading_thread.start()
    app.run(host="133.68.17.14",port=80,threaded=True)
    data_loading_thread.join()

