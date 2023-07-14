"""
FlaskでサーバをたててWeb上で地図を表示する
"""
from flask import Flask, render_template, request, jsonify
import db, pathSearch

app = Flask(__name__)

@app.route("/")
def leafletMap():
    return render_template("map.html")

@app.route('/process_ajax', methods=['POST'])
def process_ajax():
    if request.method == 'POST':
        #リクエストからデータを取得
        data = request.get_json()  
        points = data['points']

        #データベースから道路データを取得
        Db = db.yamamotoDb()
        y1, x1, y2, x2 = pathSearch.rectangleArea(points)
        Db.insertOsmRoadData(y1, x1, y2, x2, 1.25)

        #経路探索
        path = pathSearch.shortestPath(points[0], points[1], Db.link, Db.length)
        #path = pathSearch.MST(Db.link, Db.length)
        #path = pathSearch.steiner(points, Db.link, Db.length)
        #path = pathSearch.traveling(points, Db.link, Db.length)
  
        return jsonify({'path': path})
    
    else:
        return jsonify({'message': 'Invalid request method'})
    
if __name__ == "__main__":
    app.run(host="0.0.0.0",port=80)

