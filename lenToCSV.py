import csv
import networkx as nx
import db, pathSearch

# データを用意
points = [[35.18377024562999, 136.9249248504639], [35.16335399687701, 136.96702480316165]]
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