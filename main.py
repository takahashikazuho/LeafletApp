"""
FlaskでサーバをたててWeb上で地図を表示する
"""
from flask import Flask, render_template, request, jsonify
import db, pathSearch
import threading
import csv

app = Flask(__name__)

dic = {}
def load_data_csv():
    with open('data.csv', 'r', newline='') as file:
        reader = csv.reader(file, delimiter='\t')
        l = [row for row in reader]
        for elem in l:
            if elem[0] in dic.keys():
                dic[elem[0]][elem[1]] = elem[2]
            else:
                dic[elem[0]] = {elem[1] : elem[2]}
    print(dic['[35.1635255, 136.9400303]']['[35.1636073, 136.9394786]'])

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
        # print(points)

        if startPoint:
            points.append(startPoint)
        if endPoint:
            points.append(endPoint)

        #データベースから道路データを取得
        y1, x1, y2, x2 = pathSearch.rectangleArea(points)
        link, length = db.getRectangleRoadData(y1, x1, y2, x2)

        #経路探索
        path, len, _ = pathSearch.travelingPath(points, link, length)
  
        return jsonify({'path': path, 'len': len})
    
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
        path, len, points_SRP, positions_SRP, path_positions, len_walk = pathSearch.sharedRidePath(points, link, length, moveDist, value)

        return jsonify({'path': path, 'len': len, 'points_SRP': points_SRP, 'positions_SRP': positions_SRP, 'path_positions': path_positions, 'len_walk': len_walk})
    
    else:
        return jsonify({'message': 'Invalid request method'})

if __name__ == "__main__":
    data_loading_thread = threading.Thread(target=load_data_csv)
    data_loading_thread.start()
    app.run(host="133.68.17.14",port=80,threaded=True)
    data_loading_thread.join()

