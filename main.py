"""
FlaskでサーバをたててWeb上で地図を表示する
"""
from flask import Flask, render_template, request, jsonify
import db, pathSearch

app = Flask(__name__)

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

        if startPoint:
            points.append(startPoint)
        if endPoint:
            points.append(endPoint)

        #データベースから道路データを取得
        Db = db.yamamotoDb()
        y1, x1, y2, x2 = pathSearch.rectangleArea(points)
        Db.insertOsmRoadData(y1, x1, y2, x2, 1.25)

        #経路探索
        path, len, _ = pathSearch.travelingPath(points, Db.link, Db.length)
  
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

        if startPoint:
            points.append(startPoint)
        if endPoint:
            points.append(endPoint)

        #データベースから道路データを取得
        Db = db.yamamotoDb()
        y1, x1, y2, x2 = pathSearch.rectangleArea(points)
        Db.insertOsmRoadData(y1, x1, y2, x2, 1.25)

        #経路探索
        path, len = pathSearch.sharedRidePath(points, Db.link, Db.length, moveDist)
  
        return jsonify({'path': path, 'len': len})
    
    else:
        return jsonify({'message': 'Invalid request method'})

if __name__ == "__main__":
    app.run(host="133.68.17.14",port=80)

