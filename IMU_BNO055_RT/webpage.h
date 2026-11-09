#ifndef RIMU_WEBPAGE_H
#define RIMU_WEBPAGE_H

static const char WEBPAGE_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RIMU</title>
<style>
body{font-family:Arial,sans-serif;max-width:560px;margin:20px auto;padding:0 12px;color:#222}
h1{margin-bottom:8px}.card{border:1px solid #ccc;border-radius:6px;padding:12px;margin:10px 0}
button{font-size:16px;margin:4px;padding:8px 12px}button.on{background:#2b6cb0;color:#fff;border-color:#2b6cb0}
.ok{color:green}.bad{color:#b00020}
pre{white-space:pre-wrap;margin:0;font-size:14px}
#log{max-height:220px;overflow:auto;background:#f6f6f6;padding:8px;border-radius:4px;font-size:13px}
#log div{white-space:pre-wrap}
</style>
</head>
<body>
<h1 id="title">RIMU</h1>
<div class="card">Leg: <strong id="leg">-</strong><br>WiFi: <strong id="wifi">checking...</strong><br><span id="ip"></span><br>Mode: <strong id="mode">-</strong></div>
<div class="card">
<button id="b_i" onclick="sendCommand('i')">IMU cal</button>
<button id="b_s" onclick="sendCommand('s')">Still cal</button>
<button id="b_w" onclick="sendCommand('w')">Walk cal</button>
<button id="b_f" onclick="sendCommand('f')">FSM mode</button>
<button onclick="showParams()">Show params</button>
</div>
<div class="card"><strong>Real-time data</strong><pre id="live">checking...</pre></div>
<div class="card"><strong>Status</strong><div id="log">waiting...</div></div>
<div class="card"><strong>Parameters</strong><pre id="params">Press "Show params".</pre></div>
<script>
async function getJson(url){const response=await fetch(url);if(!response.ok)throw Error(response.status);return response.json();}
const MODE_BUTTON={'IMU_CAL':'b_i','STILL':'b_s','WALK':'b_w','FSM':'b_f','ADV_CAL':'','ADV':''};
let lastLog='';
function highlightMode(mode){
  for(const id of Object.values(MODE_BUTTON)){const b=document.getElementById(id);if(b)b.classList.remove('on');}
  const id=MODE_BUTTON[mode];if(id){const b=document.getElementById(id);if(b)b.classList.add('on');}
}
async function refresh(){
  try{
    const d=await getJson('/api/status');
    const wifi=document.getElementById('wifi');
    wifi.textContent=d.wifi+(d.ip?' ('+d.ip+')':'');
    wifi.className=d.connected?'ok':'bad';
    document.getElementById('ip').textContent=d.hostname;
    if(d.leg)document.getElementById('leg').textContent=d.leg;
    if(d.device){document.getElementById('title').textContent=d.device;document.title=d.device;}
    document.getElementById('mode').textContent=d.mode;
    highlightMode(d.mode);
    document.getElementById('live').textContent=
      'Raw pitch: '+d.rawPitch.toFixed(2)+' deg\n'+
      'Filtered pitch: '+d.filteredPitch.toFixed(2)+' deg\n'+
      'Angular speed: '+d.omega.toFixed(2)+' deg/s\n'+
      'FSM state: '+(d.footOffGround?'FOOT_OFF_GROUND':'FOOT_ON_GROUND');
    const log=(d.log||[]).join('\n');
    if(log!==lastLog){
      lastLog=log;
      const el=document.getElementById('log');
      el.textContent='';
      for(const line of (d.log||[])){
        const div=document.createElement('div');div.textContent=line;el.appendChild(div);
      }
      el.scrollTop=el.scrollHeight;
    }
  }catch(e){document.getElementById('wifi').textContent='offline';document.getElementById('wifi').className='bad';}
}
async function sendCommand(command){
  try{await getJson('/api/command?c='+command);refresh();}
  catch(e){alert('Command failed');}
}
async function showParams(){
  try{await getJson('/api/command?c=d');const d=await getJson('/api/params');document.getElementById('params').textContent=JSON.stringify(d,null,2);}
  catch(e){document.getElementById('params').textContent='Unable to read parameters';}
}
refresh();setInterval(refresh,1000);
</script>
</body>
</html>
)rawliteral";

#endif
