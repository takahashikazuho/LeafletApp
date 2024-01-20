import csv
import networkx as nx
import db, pathSearch

# データを用意
points = [[35.18441016255083, 136.9190883636475], [35.15573127979675, 136.97341918945315]]
y1, x1, y2, x2 = pathSearch.rectangleArea(points)
link, length = db.getRectangleRoadData(y1, x1, y2, x2)
G = pathSearch.linkToGraph(link, length)
length = dict(nx.all_pairs_dijkstra_path_length(G))
# CSVファイルにデータを書き込む
print("start")
with open('data.csv', 'w', newline='') as file:
    writer = csv.writer(file, delimiter='\t')  # タブを区切り文字として指定
    for u in length.keys():
        for v in length[u].keys():
            data = [u, v, length[u][v]]
            writer.writerow(data)
print("finish")