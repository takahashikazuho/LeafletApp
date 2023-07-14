"""
データベースアクセスを行う
"""
import psycopg2

class yamamotoDb:
    DBNAME = "osm_road_db"
    USER = "postgres"
    PASS = "usadasql"
    URL = "rain2.yamamoto.nitech.ac.jp"
    PORT = 5432
    
    def __init__(self):
        self.linkId = []
        self.link = []
        self.sourceId = []
        self.targetId = []
        self.length = []
        self.conn = None
        self.cur = None

    #矩形範囲の道路データを取得する
    #(y1, x1):左上の緯度経度 (y2, x2):右下の緯度経度 n:範囲を拡大する倍率
    def insertOsmRoadData(self, y1, x1, y2, x2, n = 1):
        x = abs(x2-x1)
        y = abs(y2-y1)
        x1 = x1-x*(n-1)
        x2 = x2+x*(n-1)
        y1 = y1+y*(n-1)
        y2 = y2-y*(n-1)

        self.conn = psycopg2.connect(database=self.DBNAME, user=self.USER, password=self.PASS, host=self.URL, port=self.PORT)
        self.cur = self.conn.cursor()

        try:
            statement = "select  "\
                            "id, osm_name, osm_source_id, osm_target_id, clazz, source, target, km, cost, x1, y1, x2, y2, geom_way  "\
                        "from "\
                            "osm_japan_car_2po_4pgr  "\
                        "where "\
                            "st_intersects("\
                                "st_geomFromText("\
                                    "'polygon(("+str(x1)+" "+str(y2)+","+str(x2)+" "+str(y2)+","+str(x2)+" "+str(y1)+","+str(x1)+" "+str(y1)+","+str(x1)+" "+str(y2)+"))', 4326),"\
                                "geom_way) "\
                            ""
            self.cur.execute(statement)
            docs = self.cur.fetchall()

            for rs in docs:
                self.linkId.append(int(rs[0]))
                self.sourceId.append(int(rs[5]))
                self.targetId.append(int(rs[6]))
                self.link.append([[float(rs[10]), float(rs[9])], [float(rs[12]), float(rs[11])]])
                self.length.append(float(rs[7]))

        except Exception as e:
            print(e)

        self.cur.close()
        self.conn.close()