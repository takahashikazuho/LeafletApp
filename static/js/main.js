var mapElement = document.getElementById('map');

// 初期座標
var center = [35.16066808015237, 136.92640764889583];
// 初期ズームレベル
var zoom_level = 14;

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
var markerType = "location";
var startMarkerFlag = false;
var endMarkerFlag = false;
var button1Active = false;

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
      break;
    case "location":
      L.marker(e.latlng).on('click', onMarkerClick).addTo(map);
      points.push([e.latlng.lat, e.latlng.lng]);
      break;
    case "end":
      if(endMarkerFlag) {
          break;
        }
        endMarker = L.marker(e.latlng,{ icon: redIcon }).on('click', onEndMarkerClick).addTo(map)
                      .bindPopup('到着地点',{autoClose:false}).openPopup();
        endPoint = [e.latlng.lat, e.latlng.lng];
        endMarkerFlag = true;
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
  console.log(index);
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

const button1Text = document.getElementById('button1Text');
// 巡回経路探索ボタンのクリックイベント
$('#btn_TSP').click(function() {
  button1Active = !button1Active; // 状態を反転させる

  if (button1Active) {
    $(this).addClass('active'); // ボタンが押されている表示にする
    var requestData = {
            points:points,
            startPoint:startPoint,
            endPoint:endPoint
        };
    $.ajax({
        url: '/process_ajax',
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
            button1Text.textContent = '経路長：'+len_round+'km';
            $('#button1Text').removeClass('hidden');
        }
    });
  } else {
    $(this).removeClass('active'); // ボタンが押されていない表示にする
    if (polyline) {
      map.removeLayer(polyline); // polyline を地図から削除
    }
    $('#button1Text').addClass('hidden');
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
