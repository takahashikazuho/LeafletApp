var mapElement = document.getElementById('map');

// 初期座標
var center = [35.174439744015935, 136.94621086120608];
// 初期ズームレベル
var zoom_level = 14;

var map = L.map(mapElement,{closePopupOnClick: false}).setView(center, zoom_level);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors',
  maxZoom: 18
}).addTo(map);
L.control.scale({imperial:false}).addTo(map);

var coordinates = [[35.19892125913606, 136.89591407775882], [35.15163514034506, 136.99831008911136]]
// L.rectangle(coordinates, { color: "#ff7800", weight: 5 , fill: false }).addTo(map);

const redIcon = L.icon({
  iconUrl: "https://esm.sh/leaflet@1.9.2/dist/images/marker-icon.png",
  iconRetinaUrl: "https://esm.sh/leaflet@1.9.2/dist/images/marker-icon-2x.png",
  shadowUrl: "https://esm.sh/leaflet@1.9.2/dist/images/marker-shadow.png",
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  tooltipAnchor: [16, -28],
  shadowSize: [41, 41],
  className: "icon-red",
});

var points = [];
var startPoint = [];
var endPoint = [];
var markers = [];
var polylines = [];
var circleMarkers = [];
var circleLayers = [];
var markerType = "location";
var startMarkerFlag = false;
var endMarkerFlag = false;
var btn_TSP_isActive = false;
var btn_SRP_isActive = false;
var btn_random_isActive = false;
var btn_test_isActive = false;

// マウスクリックで緯度経度の取得とマーカー設置
function onMapClick(e) {
  switch(markerType) {
    case "start":
      if(startMarkerFlag) {
        break;
      }
      startMarker = L.marker(e.latlng,{ icon: redIcon }).on('click', onStartMarkerClick).addTo(map)
                      .bindPopup(' start',{autoClose:false}).openPopup();
      startPoint = [e.latlng.lat, e.latlng.lng];
      startMarkerFlag = true;
      markers.push(marker);
      break;
    case "location":
      marker = L.marker(e.latlng).on('click', onMarkerClick).addTo(map);
      points.push([e.latlng.lat, e.latlng.lng]);
      markers.push(marker);
      break;
    case "end":
      if(endMarkerFlag) {
        break;
      }
      endMarker = L.marker(e.latlng,{ icon: redIcon }).on('click', onEndMarkerClick).addTo(map)
                    .bindPopup('  end',{autoClose:false}).openPopup();
      endPoint = [e.latlng.lat, e.latlng.lng];
      endMarkerFlag = true;
      markers.push(marker);
      break;
    default:
      break;
  }
}
map.on('click', onMapClick);

//マーカーをクリックしたら削除
function onMarkerClick(e) {
  var index = points.findIndex( item => JSON.stringify( item ) ===
                                  JSON.stringify([e.target.getLatLng().lat, e.target.getLatLng().lng]));
  if (index > -1){
    points.splice(index, 1)
  }
  map.removeLayer(e.target);
}

function onStartMarkerClick(e) {
  startPoint = [];
  map.removeLayer(e.target);
  startMarkerFlag = false;
}

function onEndMarkerClick(e) {
  endPoint = [];
  map.removeLayer(e.target);
  endMarkerFlag = false;
}

//testボタン(集合被覆問題)
const btn_test_text = document.getElementById('btn_test_text');
const moveDist = document.getElementById('moveDist');
$('#btn_test').click(function() {
  btn_test_isActive = !btn_test_isActive;
  if (btn_test_isActive) {
    $(this).addClass('active');
    var requestData = {
            points:points,
            moveDist:moveDist.value / 1000
        };
    $.ajax({
        url: '/test',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify(requestData),
        success: function(response) {
            var position = response.position;
            var exec_time_sec = response.exec_time_sec;  

            // 旧円を削除（重なり防止のため毎回消す）
            circleLayers.forEach(function(circle) {
                map.removeLayer(circle);
            });
            circleLayers = [];

            points.forEach(function(pt) {
                var circle = L.circle([pt[0], pt[1]], {
                    radius: Number(moveDist.value),    // 単位: メートル
                    color: 'green',
                    fillColor: 'green',
                    fillOpacity: 0.2      // 半透明
                }).addTo(map);
                circleLayers.push(circle);
            });

            // 実行時間を小数点3位まで四捨五入
            var exec_time_round = Math.round(exec_time_sec * 1000) / 1000;

            var html = 'Exec time：' + exec_time_round + ' sec';
            btn_test_text.innerHTML = html;

            $('#btn_test_text').removeClass('hidden');
            var option = {
              radius: 5,       
              fillColor: "blue", 
              color: "blue",     
              opacity: 1,         
              fillOpacity: 1
            }
            circleMarkers = [];
            for(var i=0; i<position.length; i++) {
              circleMarkers[i] = L.circleMarker(position[i], option).addTo(map);
            }
        }
    });
  } else {
    $(this).removeClass('active'); // ボタンが押されていない表示にする
    circleLayers.forEach(function(circle) {
                map.removeLayer(circle);
    });
    for(var i=0; i<circleMarkers.length; i++) {
        map.removeLayer(circleMarkers[i]);
    }
    circleMarkers = [];
    $('#btn_test_text').addClass('hidden');
  }

});

const btn_TSP_text = document.getElementById('btn_TSP_text');
// 経路探索ボタンのクリックイベント
$('#btn_TSP').click(function() {
  btn_TSP_isActive = !btn_TSP_isActive; // 状態を反転させる

  if (btn_TSP_isActive) {
    $(this).addClass('active'); // ボタンが押されている表示にする
    let elements = document.getElementsByName('type-traveling');
    let checkValue = '';
    for (let i = 0; i < elements.length; i++){
        if (elements.item(i).checked){
            checkValue = elements.item(i).value;
        }
    }
    var requestData = {
            points:points,
            startPoint:startPoint,
            endPoint:endPoint,
            value:checkValue
        };
    $.ajax({
        url: '/TSP_path',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify(requestData),
        success: function(response) {
            var path = response.path;
            var len = response.len;
            var position = response.position;
            var exec_time_sec = response.exec_time_sec;  // 実行時間を取得
            var percent = response.percent;

            // 経路を表示
            polyline = L.polyline(path, { color: 'red' })
            polyline.addTo(map);

            var len_round = Math.round(len * Math.pow(10, 3) ) / Math.pow(10, 3);

            // 実行時間を小数点3位まで四捨五入
            var exec_time_round = Math.round(exec_time_sec * 1000) / 1000;

            var html = 
              'Length：' + len_round + 'km<br>' +
              'Exec time：' + exec_time_round + ' sec';

              if(percent !== undefined && percent !== null) {
                // 0.00001未満なら指数表記、そうでなければ小数点以下5桁
                var percent_display = (Math.abs(percent) < 0.00001) ? percent.toExponential(5) : percent.toFixed(5);
                html += '<br>Percent：' + percent_display + '%';
              }
            btn_TSP_text.innerHTML = html;

            $('#btn_TSP_text').removeClass('hidden');
            //移動先の点を表示
            var option = {
              radius: 5,       
              fillColor: "blue", 
              color: "blue",     
              opacity: 1,         
              fillOpacity: 1
            }
            circleMarkers = [];
            for(var i=0; i<position.length; i++) {
              circleMarkers[i] = L.circleMarker(position[i], option).addTo(map);
            }
        }
    });
  } else {
    $(this).removeClass('active'); // ボタンが押されていない表示にする
    if (polyline) {
      map.removeLayer(polyline); // polyline を地図から削除
      for(var i=0; i<circleMarkers.length; i++) {
        map.removeLayer(circleMarkers[i]);
      }
      circleMarkers = [];
    }
    $('#btn_TSP_text').addClass('hidden');
  }
});

const btn_SRP_text = document.getElementById('btn_SRP_text');
// BR探索ボタンのクリックイベント
$('#btn_SRP').click(function() {
  btn_SRP_isActive = !btn_SRP_isActive; // 状態を反転させる

  if (btn_SRP_isActive) {
    $(this).addClass('active'); // ボタンが押されている表示にする
    let elements = document.getElementsByName('type');
    let checkValue = '';
    for (let i = 0; i < elements.length; i++){
        if (elements.item(i).checked){
            checkValue = elements.item(i).value;
        }
    }
    var requestData = {
            points:points,
            startPoint:startPoint,
            endPoint:endPoint,
            moveDist:moveDist.value / 1000,
            value:checkValue
        };
    $.ajax({
        url: '/SRP_path',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify(requestData),
        success: function(response) {
            var path = response.path;
            var len = response.len;
            var positions_SRP = response.positions_SRP;
            var path_positions = response.path_positions;
            var len_walk = response.len_walk;
            var time = Math.round(response.time * 1000) / 1000;

            // 経路を表示
            polyline2 = L.polyline(path, { color: 'blue' })
            polyline2.addTo(map);
            for(var i=0; i<path_positions.length; i++) {
              polylines[i] = L.polyline(path_positions[i], { color: 'green', opacity: 0.8 })
              polylines[i].addTo(map);
            }
            len = Math.round(len * Math.pow(10, 3) ) / Math.pow(10, 3);
            len_walk = Math.round(len_walk * Math.pow(10, 3) ) / Math.pow(10, 3);
            btn_SRP_text.textContent = 'Traveling：' + len + 'km\nWalking：' + len_walk + 'km\nExec time：' + time + 's';
            $('#btn_SRP_text').removeClass('hidden');

            //移動先の点を表示
            var option = {
              radius: 5,       
              fillColor: "blue", 
              color: "blue",     
              opacity: 1,         
              fillOpacity: 1
            }
            circleMarkers = [];
            for(var i=0; i<positions_SRP.length; i++) {
              circleMarkers[i] = L.circleMarker(positions_SRP[i], option).addTo(map);
            }
        }
    });
  } else {
    $(this).removeClass('active'); // ボタンが押されていない表示にする
    if (polyline2) {
      map.removeLayer(polyline2); // polyline2 を地図から削除
      for(var i=0; i<polylines.length; i++) {
        map.removeLayer(polylines[i]);
      }
      for(var i=0; i<circleMarkers.length; i++) {
        map.removeLayer(circleMarkers[i]);
      }
      circleMarkers = [];
    }
    for(var i=0; i<markers.length; i++) {
      markers[i].closePopup();
    }
    $('#btn_SRP_text').addClass('hidden');
  }
});

//ランダムボタン
const random_num = document.getElementById('random_num');
$('#btn_random').click(function() {
  btn_random_isActive = !btn_random_isActive; // 状態を反転させる

  if (btn_random_isActive) {
    $(this).addClass('active'); // ボタンが押されている表示にする
    var bounds = calculateBounds(coordinates);
    for (var i = 0; i < random_num.value; i++) {
      var randomLat = bounds.getSouth() + Math.random() * (bounds.getNorth() - bounds.getSouth());
      var randomLng = bounds.getWest() + Math.random() * (bounds.getEast() - bounds.getWest());

      marker = L.marker([randomLat, randomLng]).addTo(map);
      points.push([randomLat, randomLng]);
      markers.push(marker);
    }
  } else {
    $(this).removeClass('active'); // ボタンが押されていない表示にする
    if (markers) {
      for(var i=0; i<markers.length; i++) {
        map.removeLayer(markers[i]);
      }
      points = []
    }
  }
});

function calculateBounds(coords) {
  var minLat = Infinity, maxLat = -Infinity, minLng = Infinity, maxLng = -Infinity;
  coords.forEach(function(coord) {
      minLat = Math.min(minLat, coord[0]);
      maxLat = Math.max(maxLat, coord[0]);
      minLng = Math.min(minLng, coord[1]);
      maxLng = Math.max(maxLng, coord[1]);
  });
  return L.latLngBounds([minLat, minLng], [maxLat, maxLng]);
};

var mamlistboxdiv;
var mamlistboxa;
var mamlistbox;
var mamlistbox_active=false;
window.addEventListener("load",function(){
  mamlistboxdiv=document.querySelector(".mamListBox>a>div");
  mamlistboxa=document.querySelector(".mamListBox>a");
  mamlistbox=document.querySelector(".mamListBox>select");
  mamlistboxa.addEventListener("click",function(event){
    if(mamlistbox_active==false){
      mamlistbox.style.display = "block";
      mamlistbox_active=true;
      mamlistbox.focus();
    }else{
      mamlistbox_active=false;
    }
  });
  mamlistbox.addEventListener("blur",function(){
    mamlistbox.style.display = "none";
  });
  mamlistbox.addEventListener("click",function(){
    mamlistboxdiv.innerHTML = mamlistbox.querySelectorAll('option')[mamlistbox.selectedIndex].innerHTML;
    mamlistbox_active=false;
    mamlistbox.blur();
    markerType = mamlistbox.value;
  });
  document.documentElement.addEventListener("click",mamListboxOtherClick);
});
function mamListboxOtherClick(event){
  if(event.target==mamlistboxdiv){return;}
  if(event.target==mamlistboxa){return;}
  if(event.target==mamlistbox){return;}
  mamlistbox_active=false;
}
