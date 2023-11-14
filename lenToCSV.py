import csv
import networkx as nx
import db, pathSearch

# データを用意
points = [[35.16966936723668, 136.90265178680423], [35.15224248922456, 136.95080280303958]]
y1, x1, y2, x2 = pathSearch.rectangleArea(points)
link, length = db.getRectangleRoadData(y1, x1, y2, x2)
G = pathSearch.linkToGraph(link, length)
length = dict(nx.all_pairs_dijkstra_path_length(G))
# CSVファイルにデータを書き込む
with open('data.csv', 'w', newline='') as file:
    writer = csv.writer(file, delimiter='\t')  # タブを区切り文字として指定
    for u in length.keys():
        for v in length[u].keys():
            data = [u, v, length[u][v]]
            writer.writerow(data)