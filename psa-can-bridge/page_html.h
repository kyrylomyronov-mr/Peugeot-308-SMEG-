#pragma once

const char page_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang='en'>
<head>
  <meta charset='utf-8'/>
  <meta name='viewport' content='width=device-width,initial-scale=1'/>
  <title>PSA CAN Bridge</title>
  <style>
    :root { 
      --bg: #020617; --card: rgba(30, 41, 59, 0.7); --text: #f8fafc; 
      --blue: #3b82f6; --red: #ef4444; --green: #22c55e; --purple: #818cf8; --gray: #334155;
      --accent: #ccff00; /* Peugeot Neon Lime */
    }
    body { font-family: 'Inter', system-ui, sans-serif; background: radial-gradient(circle at top right, #0f172a, #020617); color: var(--text); margin: 0; padding: 15px; line-height: 1.5; min-height: 100vh; }
    
    .card { 
      background: var(--card); backdrop-filter: blur(10px); border-radius: 20px; padding: 24px; 
      box-shadow: 0 20px 25px -5px rgba(0,0,0,0.5); max-width: 650px; margin: 0 auto 20px auto; 
      border: 1px solid rgba(255,255,255,0.1); transition: transform 0.3s ease;
    }
    h3 { margin-top: 0; font-size: 16px; text-transform: uppercase; letter-spacing: 1px; color: #94a3b8; border-bottom: 2px solid var(--accent); display: inline-block; padding-bottom: 4px; margin-bottom: 20px; }
    
    /* Peugeot "Piano Key" Buttons */
    button, .btn { 
      display: inline-flex; align-items: center; justify-content: center;
      background: #1e293b; color: #cbd5e1; border: 1px solid #334155; padding: 14px 20px; 
      border-radius: 12px; cursor: pointer; font-weight: 700; font-size: 13px;
      transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1); gap: 10px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.3);
    }
    button:hover { border-color: var(--accent); color: var(--accent); background: #0f172a; transform: translateY(-2px); }
    button:active { transform: translateY(0); filter: brightness(0.9); }
    
    .btn-blue { border-color: var(--blue); color: var(--blue); }
    .btn-red { border-color: var(--red); color: var(--red); }
    .btn-green { background: var(--accent); color: #000; border: none; }
    .btn-purple { border-color: var(--purple); color: var(--purple); }
    .btn-orange { border-color: #f59e0b; color: #f59e0b; }
    
    .grid { display: grid; gap: 8px; margin-top: 15px; }
    .grid-2 { grid-template-columns: repeat(2, 1fr); }
    .grid-3 { grid-template-columns: repeat(3, 1fr); }
    .grid-4 { grid-template-columns: repeat(4, 1fr); }

    /* Mobile Overrides (Phone/iPhone) */
    @media (max-width: 480px) {
      body { padding: 10px; }
      .card { padding: 16px; border-radius: 16px; width: 100%; box-sizing: border-box; }
      button, .btn { padding: 12px 10px; font-size: 11px; letter-spacing: 0; }
      h3 { font-size: 13px; margin-bottom: 15px; }
      .grid-4 { grid-template-columns: repeat(2, 1fr); } /* Drop 4 to 2 on small screens */
    }

    /* Modern Tab Styling */
    .tab-bar { display: flex; gap: 4px; max-width: 650px; margin: 0 auto 24px auto; background: rgba(30, 41, 59, 0.5); padding: 4px; border-radius: 16px; border: 1px solid #334155; }
    .tab-btn { flex: 1; padding: 12px; border: none; background: transparent; color: #64748b; font-size: 12px; letter-spacing: 0.5px; border-radius: 12px; cursor: pointer; box-shadow: none; }
    .tab-btn.active { background: var(--accent); color: #000; font-weight: 800; }
    .tab-content { display: none; animation: slideUp 0.4s ease-out; margin-bottom: 50px; }
    .tab-content.active { display: block; }
    @keyframes slideUp { from { opacity: 0; transform: translateY(10px); } to { opacity: 1; transform: translateY(0); } }

    /* Custom Elements */
    .log-box { background: rgba(2, 6, 23, 0.8); padding: 16px; border-radius: 16px; font-family: 'JetBrains Mono', monospace; font-size: 11px; height: 500px; overflow-y: auto; border: 1px solid #334155; color: #94a3b8; }
    input[type='range'] { width: 100%; accent-color: var(--accent); cursor: pointer; }
    select, input[type='time'], input[type='text'] { background: #0f172a; color: #fff; border: 1px solid #334155; padding: 10px; border-radius: 10px; font-size: 13px; width: 100%; transition: border-color 0.2s; }
    select:focus { border-color: var(--accent); outline: none; }
  </style>
  <script>
    let logData = [];
    
    async function updateLog() {
      try {
        let r = await fetch('/log');
        if (!r.ok) return;
        let arr = await r.json();
        logData = arr; 
        
        let h0 = ''; let h1 = '';
        let c0 = 0; let c1 = 0;
        
        [...arr].reverse().forEach(x => {
          let idHex = x.id.toString(16).toUpperCase().padStart(3, '0');
          let line = "<div style='border-bottom:1px solid #333; padding:2px 0;'>[" + x.t + "] (c:" + x.c + ") 0x" + idHex + " | [" + x.dlc + "] " + x.d + "</div>";
          if (String(x.c) === "1") { h0 += line; c0++; }
          else { h1 += line; c1++; }
        });
        
        document.getElementById('logBox0').innerHTML = h0 || 'No frames';
        document.getElementById('logBox1').innerHTML = h1 || 'No frames';
        document.getElementById('cnt0').innerText = c0;
        document.getElementById('cnt1').innerText = c1;
      } catch (e) {}
    }

    function copyLog(cIdx) {
      let filtered = logData.filter(x => cIdx === -1 || (cIdx === 0 && String(x.c) === "1") || (cIdx === 1 && String(x.c) === "0"));
      let text = filtered.map(x => {
        let src = String(x.c) === "1" ? "CAR" : "RAD";
        return "[" + x.t + "] " + src + " | 0x" + x.id.toString(16).toUpperCase().padStart(3,'0') + " | [" + x.dlc + "] " + x.d;
      }).reverse().join('\r\n');
      navigator.clipboard.writeText(text).then(() => alert('Copied!'));
    }

    async function sendCtrl(idx) {
      await fetch('/ctrl?idx=' + idx);
    }

    async function sendEmf(id) {
      await fetch('/emf?id=' + id);
    }

    function openTab(e, tabId) {
      document.querySelectorAll('.tab-content').forEach(x => x.classList.remove('active'));
      document.querySelectorAll('.tab-btn').forEach(x => x.classList.remove('active'));
      let el = document.getElementById(tabId);
      if (el) el.classList.add('active');
      if (e && e.currentTarget) e.currentTarget.classList.add('active');
    }


    function updateSim() {
      let park = document.getElementById('chkPark').checked ? 1 : 0;
      let url = "/sim?park=" + park;
      for (let i = 0; i < 6; i++) {
        let val = document.getElementById('p' + i).value;
        document.getElementById('v_p' + i).innerText = val;
        url += "&p" + i + "=" + val;
      }
      fetch(url);
    }

    async function updateEco() {
      let b = document.getElementById('chkEcoBypass').checked ? 1 : 0;
      let f = document.getElementById('chkEcoForce').checked ? 1 : 0;
      await fetch("/sim?eco_b=" + b + "&eco_f=" + f);
    }

    async function syncClock() {
      let n = new Date();
      let url = "/clock?y=" + n.getFullYear() + "&m=" + (n.getMonth()+1) + "&d=" + n.getDate() + "&h=" + n.getHours() + "&min=" + n.getMinutes();
      await fetch(url);
      document.getElementById('clkStat').innerText = 'Clock Synced!';
    }

    async function updateConfig() {
      let lang = document.getElementById('selLang').value;
      let temp = document.getElementById('selTemp').value;
      let drl = document.getElementById('chkNativeDRL').checked ? 1 : 0;
      let park = document.getElementById('chkNativePark').checked ? 1 : 0;
      let grille = document.getElementById('chkNativeGrille').checked ? 1 : 0;

      let url = "/config?lang=" + lang + "&temp=" + temp + "&drl=" + drl + "&park=" + park + "&grille=" + grille;
      await fetch(url);
      alert('Config Saved!');
    }

    async function loadConfig() {
      try {
        let r = await fetch('/config');
        if (r.ok) {
          let c = await r.json();
          document.getElementById('selLang').value = c.lang;
          document.getElementById('selTemp').value = c.temp;
          document.getElementById('chkNativeDRL').checked = (c.drl === 1);
          document.getElementById('chkNativePark').checked = (c.park === 1);
          document.getElementById('chkNativeGrille').checked = (c.grille === 1);
          document.getElementById('chkEcoBypass').checked = (c.eco_b === 1);
        }
      } catch (e) {}
    }

    async function setManualTime() {
      let val = document.getElementById('manTime').value;
      if (!val) return;
      let [h, m] = val.split(':');
      let n = new Date();
      await fetch("/clock?y=" + n.getFullYear() + "&m=" + (n.getMonth()+1) + "&d=" + n.getDate() + "&h=" + parseInt(h) + "&min=" + parseInt(m));
      document.getElementById('clkStat').innerText = 'Time Updated!';
    }
    
    window.addEventListener('DOMContentLoaded', loadConfig);
    setInterval(updateLog, 2000);
    setInterval(loadConfig, 1000); // Polling for 15B Sniffer
    setInterval(async () => {
      let r = await fetch('/clock');
      if (r.ok) {
        let text = await r.text();
        document.getElementById('liveClock').innerText = text;
      }
    }, 1000);
  </script>
</head>
<body>
  <div class='tab-bar'>
    <button class='tab-btn active' onclick='openTab(event, "dashboard")'>📊 DRIVE</button>
    <button class='tab-btn' onclick='openTab(event, "settings")'>⚙️ SYSTEM</button>
    <button class='tab-btn' onclick='openTab(event, "console")'>📟 CONSOLE</button>
  </div>

  <!-- Tab 1: Dashboard -->
  <div id='dashboard' class='tab-content active'>
    <div class='card'>
      <h3>📻 MULTIMEDIA (0x122)</h3>
      <div class='grid grid-4'>
        <button class='btn-blue' onclick='sendCtrl(19)'>📻 RADIO</button>
        <button class='btn-blue' onclick='sendCtrl(20)'>📞 PHONE</button>
        <button class='btn-blue' onclick='sendCtrl(18)'>🗺️ NAVI</button>
        <button class='btn-blue' onclick='sendCtrl(17)'>⚙️ SETUP</button>
        <button onclick="sendCtrl(29)">🧮 APPS</button>
        <button onclick="sendCtrl(30)">🚗 TRIP</button>
        <button onclick="sendCtrl(32)">❄️ CLIP</button>
      </div>
    </div>

    <div class='card'>
      <h3>📺 EMF DISPLAY (0x3E5)</h3>
      <div style='display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; max-width: 280px; margin: 15px auto;'>
        <div></div> <button class='btn-orange' onclick='sendEmf(6)'>🔼</button> <div></div>
        <button class='btn-orange' onclick='sendEmf(9)'>◀️</button> <button class='btn-green' onclick='sendEmf(4)'>OK</button> <button class='btn-orange' onclick='sendEmf(8)'>▶️</button>
        <div></div> <button class='btn-orange' onclick='sendEmf(7)'>🔽</button> <div></div>
      </div>
      <div class='grid grid-2'>
        <button class='btn-blue' onclick='sendEmf(1)'>MENU</button>
        <button class='btn-red' onclick='sendEmf(5)'>ESC</button>
      </div>
    </div>
  </div>

  <!-- Tab 2: Settings -->
  <div id='settings' class='tab-content'>
    <div class='card'>
      <h3>⚙️ CONFIGURATION</h3>
      <div class='grid grid-2'>
        <div style='display:flex; flex-direction:column; gap:5px;'>
          <label style='font-size:11px; color:#64748b; text-transform:uppercase;'>Language</label>
          <select id='selLang'>
            <option value='14'>Русский</option>
            <option value='1'>English</option>
            <option value='0'>Français</option>
          </select>
        </div>
        <div style='display:flex; flex-direction:column; gap:5px;'>
          <label style='font-size:11px; color:#64748b; text-transform:uppercase;'>Unit</label>
          <select id='selTemp'>
            <option value='1'>°C</option>
            <option value='0'>°F</option>
          </select>
        </div>
      </div>
      <div class='grid'>
        <button class='btn-green' onclick='updateConfig()'>💾 SAVE CHANGES</button>
        <button class='btn-purple' onclick="fetch('/ctrl?idx=17&t=8000')">🧪 SETUP LONG PRESS (8s)</button>
      </div>

      <div style='margin-top: 25px; padding-top: 20px; border-top: 1px solid rgba(255,255,255,0.05);'>
        <div style='margin-top: 15px; padding-top: 10px; border-top: 1px solid rgba(255,255,255,0.05);'>
          <h4 style='color:var(--accent); font-size:12px;'>🔦 LIGHTS & ASSISTANCE (NATIVE)</h4>
          <div style='display: flex; flex-direction: column; gap: 10px;'>
            <label style='display:flex; align-items:center; gap:12px; cursor:pointer;'>
              <input type='checkbox' id='chkNativeDRL' onchange='updateConfig()' style='width:20px; height:20px;'> DRL / LED (D2[1])
            </label>
            <label style='display:flex; align-items:center; gap:12px; cursor:pointer;'>
              <input type='checkbox' id='chkNativePark' onchange='updateConfig()' style='width:20px; height:20px;'> Park Assist (D4[7])
            </label>
            <label style='display:flex; align-items:center; gap:12px; cursor:pointer;'>
              <input type='checkbox' id='chkNativeGrille' onchange='updateConfig()' style='width:20px; height:20px;'> Grille Light (D6[2])
            </label>
          </div>
        </div>
      </div>
    </div>

    <div class='card'>
      <h3>🕒 CLOCK SYNC <span id='liveClock' style='float:right; color:var(--accent); font-family:monospace;'>--:--:--</span></h3>
      <div class='grid grid-2'>
        <button class='btn-green' onclick='syncClock()'>🕒 SYNC PHONE</button>
        <div style='display:flex; gap:8px;'>
          <input type='time' id='manTime'>
          <button onclick='setManualTime()'>SET</button>
        </div>
      </div>
      <span id='clkStat' style='margin-top: 10px; display:block; color: #64748b; font-size: 11px;'>STATUS: IDLE</span>
    </div>

    <div class='card'>
      <h3>⚡ SIGNAL EMULATION</h3>
      <div style='background: rgba(0,0,0,0.2); padding: 15px; border-radius: 12px;'>
        <label style='display:flex; align-items:center; gap:12px; cursor:pointer; margin-bottom:15px;'>
          <input type='checkbox' id='chkPark' onchange='updateSim()' style='width:20px; height:20px;'> Parking Radar Sim (0x0E1)
        </label>
        <div style='display:grid; grid-template-columns: repeat(2, 1fr); gap: 12px; font-size: 11px;'>
          <div>REAR L: <input type='range' id='p0' min='0' max='7' value='7' oninput='updateSim()'><b id='v_p0'>7</b></div>
          <div>REAR C: <input type='range' id='p1' min='0' max='7' value='7' oninput='updateSim()'><b id='v_p1'>7</b></div>
          <div>REAR R: <input type='range' id='p2' min='0' max='7' value='7' oninput='updateSim()'><b id='v_p2'>7</b></div>
          <div>FRNT L: <input type='range' id='p3' min='0' max='7' value='7' oninput='updateSim()'><b id='v_p3'>7</b></div>
          <div>FRNT C: <input type='range' id='p4' min='0' max='7' value='7' oninput='updateSim()'><b id='v_p4'>7</b></div>
          <div>FRNT R: <input type='range' id='p5' min='0' max='7' value='7' oninput='updateSim()'><b id='v_p5'>7</b></div>
        </div>
      </div>
    </div>

    <div class='card' style='border:1px solid var(--red);'>
      <h3 style='color:var(--red); border-color:var(--red);'>🔋 POWER & BATTERY</h3>
      <div style='background: rgba(239, 68, 68, 0.1); padding: 12px; border-radius: 10px; margin-bottom: 15px; font-size: 11px; color: #fca5a5;'>
        <b>WARNING</b>: Bypassing Economy Mode will prevent the car from shutting down the radio. This may drain your battery if left active!
      </div>
      <div style='display: flex; flex-direction: column; gap: 12px;'>
        <label style='display:flex; align-items:center; gap:12px; cursor:pointer;'>
          <input type='checkbox' id='chkEcoBypass' onchange='updateEco()' style='width:20px; height:20px;'> <b>Eco-Mode Bypass</b> (Keep radio ON)
        </label>
        <label style='display:flex; align-items:center; gap:12px; cursor:pointer;'>
          <input type='checkbox' id='chkEcoForce' onchange='updateEco()' style='width:20px; height:20px;'> Force Economy Mode (Turn radio OFF)
        </label>
      </div>
    </div>
  </div>

  <!-- Tab 3: Console -->
  <div id='console' class='tab-content'>
    <div class='card' style='max-width: 1200px;'>
      <div style='display:flex; gap:15px; flex-wrap: wrap;'>
        <div style='flex: 1; min-width: 300px;'>
          <div style='display:flex; justify-content: space-between; align-items: center; margin-bottom: 12px;'>
            <h3 style='margin:0; color:#10b981;'>🚗 CAN0 (CAR) <span id='cnt0' style='font-size:11px; opacity:0.5;'>0</span></h3>
            <button class='btn' style='padding:6px 12px; font-size:10px;' onclick='copyLog(0)'>📋 COPY</button>
          </div>
          <div id='logBox0' class='log-box'>WAITING...</div>
        </div>
        <div style='flex: 1; min-width: 300px;'>
          <div style='display:flex; justify-content: space-between; align-items: center; margin-bottom: 12px;'>
            <h3 style='margin:0; color:#3b82f6;'>📻 CAN1 (RADIO) <span id='cnt1' style='font-size:11px; opacity:0.5;'>0</span></h3>
            <button class='btn' style='padding:6px 12px; font-size:10px;' onclick='copyLog(1)'>📋 COPY</button>
          </div>
          <div id='logBox1' class='log-box'>WAITING...</div>
        </div>
      </div>
    </div>
  </div>
</body>
</html>
)rawliteral";
