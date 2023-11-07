var mapElement = document.getElementById('map');

// 初期座標
var center = [35.16066808015237, 136.92640764889583];
// 初期ズームレベル
var zoom_level = 15;

var map = L.map(mapElement,{closePopupOnClick: false}).setView(center, zoom_level);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors',
  maxZoom: 18
}).addTo(map);

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
var markerType = "location";
var startMarkerFlag = false;
var endMarkerFlag = false;
var btn_TSP_isActive = false;
var btn_SRP_isActive = false;

// マウスクリックで緯度経度の取得とマーカー設置
function onMapClick(e) {
  switch(markerType) {
    case "start":
      if(startMarkerFlag) {
        break;
      }
      startMarker = L.marker(e.latlng,{ icon: redIcon }).on('click', onStartMarkerClick).addTo(map)
                      .bindPopup('出発地点',{autoClose:false}).openPopup();
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
                    .bindPopup('到着地点',{autoClose:false}).openPopup();
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

const btn_TSP_text = document.getElementById('btn_TSP_text');
// 巡回経路探索ボタンのクリックイベント
$('#btn_TSP').click(function() {
  btn_TSP_isActive = !btn_TSP_isActive; // 状態を反転させる

  if (btn_TSP_isActive) {
    $(this).addClass('active'); // ボタンが押されている表示にする
    var requestData = {
            points:points,
            startPoint:startPoint,
            endPoint:endPoint
        };
    $.ajax({
        url: '/TSP_path',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify(requestData),
        success: function(response) {
            var path = response.path;
            var len = response.len;
            // 経路を表示
            polyline = L.polyline(path, { color: 'red' })
            polyline.addTo(map);
            var len_round = Math.round(len * Math.pow(10, 3) ) / Math.pow(10, 3);
            btn_TSP_text.textContent = 'Traveling：'+len_round+'km';
            $('#btn_TSP_text').removeClass('hidden');
        }
    });
  } else {
    $(this).removeClass('active'); // ボタンが押されていない表示にする
    if (polyline) {
      map.removeLayer(polyline); // polyline を地図から削除
    }
    $('#btn_TSP_text').addClass('hidden');
  }
});

const btn_SRP_text = document.getElementById('btn_SRP_text');
const moveDist = document.getElementById('moveDist');
// 相乗り経路探索ボタンのクリックイベント
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
            moveDist:moveDist.value,
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
            var points_SRP = response.points_SRP;
            var positions_SRP = response.positions_SRP;
            var path_positions = response.path_positions;
            var len_walk = response.len_walk;

            // 経路を表示
            polyline2 = L.polyline(path, { color: 'blue' })
            polyline2.addTo(map);
            for(var i=0; i<path_positions.length; i++) {
              polylines[i] = L.polyline(path_positions[i], { color: 'green', opacity: 0.8 })
              polylines[i].addTo(map);
            }
            len = Math.round(len * Math.pow(10, 3) ) / Math.pow(10, 3);
            len_walk = Math.round(len_walk * Math.pow(10, 3) ) / Math.pow(10, 3);
            btn_SRP_text.textContent = 'Traveling：' + len + 'km\nWalking：' + len_walk + 'km';
            $('#btn_SRP_text').removeClass('hidden');

            //マーカーに巡回順を追加
            for(var i=0; i<points_SRP.length; i++) {
              points_SRP_latlng = L.latLng(points_SRP[i][0], points_SRP[i][1]);
              for(var j=0; j<markers.length; j++) {
                if(markers[j].getLatLng().equals(points_SRP_latlng)) {
                  markers[j].bindPopup(String(i+1),{autoClose:false}).openPopup();
                }
              }
            }

            //移動先の点を表示
            var option = {
              radius: 5,       
              fillColor: "blue", 
              color: "blue",     
              opacity: 1,         
              fillOpacity: 1
            }
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

    }
    for(var i=0; i<markers.length; i++) {
      markers[i].closePopup();
    }
    $('#btn_SRP_text').addClass('hidden');
  }
});

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
