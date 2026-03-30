#pragma once

const char page_html[] PROGMEM = R"rawliteral(
<!doctype html><html lang='ru'><head><meta charset='utf-8'/>
<meta name='viewport' content='width=device-width,initial-scale=1'/>
<title>PSA CAN Remote</title>
<style>
body{font-family:'Inter',system-ui,-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:0;padding:0;background:linear-gradient(135deg,#0f172a 0%,#1e293b 100%);color:#f8fafc;min-height:100vh;display:flex;flex-direction:column;}
header{background:rgba(15,23,42,0.8);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border-bottom:1px solid rgba(255,255,255,0.05);padding:20px 24px;position:sticky;top:0;z-index:10;display:flex;align-items:center;justify-content:space-between;box-shadow:0 4px 20px rgba(0,0,0,0.2);}
header h1{margin:0;font-size:20px;font-weight:600;letter-spacing:-0.02em;display:flex;align-items:center;gap:10px;}
main{max-width:1000px;margin:30px auto;padding:0 24px;display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:24px;width:100%;box-sizing:border-box;}
h3{margin:0 0 16px;font-size:16px;font-weight:600;color:#94a3b8;text-transform:uppercase;letter-spacing:0.05em;display:flex;align-items:center;gap:8px;}
.card{background:rgba(30,41,59,0.5);backdrop-filter:blur(16px);-webkit-backdrop-filter:blur(16px);border:1px solid rgba(255,255,255,0.05);border-radius:20px;padding:28px;box-shadow:0 10px 30px rgba(0,0,0,0.2);transition:transform 0.3s ease,box-shadow 0.3s ease;display:flex;flex-direction:column;}
.card:hover{transform:translateY(-2px);box-shadow:0 15px 40px rgba(0,0,0,0.3);border-color:rgba(255,255,255,0.08);}
.card p.desc{margin:0 0 20px;color:#cbd5e1;font-size:14px;line-height:1.5;}
.metrics{display:flex;flex-wrap:wrap;gap:20px;margin-bottom:20px;}
.metric{background:rgba(15,23,42,0.4);border-radius:12px;padding:16px;flex:1 1 140px;border:1px solid rgba(255,255,255,0.03);}
.metric .label{font-size:11px;color:#94a3b8;text-transform:uppercase;letter-spacing:.1em;margin-bottom:8px;font-weight:500;}
.metric .value{font-size:24px;font-weight:600;color:#f8fafc;display:flex;align-items:baseline;gap:4px;}
.metric .value.hex{font-family:'JetBrains Mono','Fira Code',monospace;font-size:16px;letter-spacing:.05em;color:#38bdf8;}
.metric .unit{font-size:12px;color:#64748b;font-weight:500;}
.badge-container{display:flex;gap:12px;align-items:center;margin-top:auto;}
.badge{display:inline-flex;align-items:center;gap:6px;padding:6px 12px;border-radius:999px;font-size:12px;font-weight:600;text-transform:uppercase;letter-spacing:0.05em;}
.badge::before{content:'';display:block;width:8px;height:8px;border-radius:50%;}
.badge.ok{background:rgba(16,185,129,0.1);color:#34d399;border:1px solid rgba(16,185,129,0.2);}
.badge.ok::before{background:#34d399;box-shadow:0 0 8px #34d399;}
.badge.err{background:rgba(239,68,68,0.1);color:#f87171;border:1px solid rgba(239,68,68,0.2);}
.badge.err::before{background:#f87171;box-shadow:0 0 8px #f87171;}
.badge.warn{background:rgba(245,158,11,0.1);color:#fbbf24;border:1px solid rgba(245,158,11,0.2);}
.badge.warn::before{background:#fbbf24;box-shadow:0 0 8px #fbbf24;}
.control-group{margin-bottom:20px;}
.toggle-wrapper{display:flex;align-items:center;justify-content:space-between;padding:12px 0;border-bottom:1px solid rgba(255,255,255,0.05);}
.toggle-wrapper:last-child{border-bottom:none;}
.toggle-label{display:flex;flex-direction:column;gap:4px;}
.toggle-title{font-size:15px;color:#f8fafc;font-weight:500;}
.toggle-desc{font-size:12px;color:#64748b;}
.switch{position:relative;display:inline-block;width:44px;height:24px;}
.switch input{opacity:0;width:0;height:0;}
.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:rgba(15,23,42,0.6);transition:.3s;border-radius:24px;border:1px solid rgba(255,255,255,0.1);}
.slider:before{position:absolute;content:'';height:18px;width:18px;left:2px;bottom:2px;background-color:#94a3b8;transition:.3s;border-radius:50%;}
input:checked + .slider{background-color:#3b82f6;border-color:#3b82f6;}
input:checked + .slider:before{transform:translateX(20px);background-color:#fff;box-shadow:0 2px 4px rgba(0,0,0,0.2);}
input:focus + .slider{box-shadow:0 0 0 2px rgba(59,130,246,0.5);}
input[type=datetime-local],input[type=number],select{width:100%;padding:12px 16px;border-radius:12px;border:1px solid rgba(255,255,255,0.1);background:rgba(15,23,42,0.4);color:#f8fafc;font-size:14px;transition:border-color 0.2s,box-shadow 0.2s;box-sizing:border-box;font-family:inherit;}
input[type=datetime-local]:focus,input[type=number]:focus,select:focus{outline:none;border-color:#3b82f6;box-shadow:0 0 0 3px rgba(59,130,246,0.2);}
select{appearance:none;background-image:url("data:image/svg+xml;charset=US-ASCII,%3Csvg%20width%3D%2224%22%20height%3D%2224%22%20viewBox%3D%220%200%2024%2024%22%20fill%3D%22none%22%20stroke%3D%22%2394a3b8%22%20stroke-width%3D%222%22%20stroke-linecap%3D%22round%22%20stroke-linejoin%3D%22round%22%3E%3Cpolyline%20points%3D%226%209%2012%2015%2018%209%22%2F%3E%3C%2Fsvg%3E");background-repeat:no-repeat;background-position:right 12px center;background-size:16px;}
select option{background:#1e293b;color:#f8fafc;}
input[type=datetime-local]::-webkit-calendar-picker-indicator{filter:invert(1);opacity:0.6;cursor:pointer;}
.input-group{margin-bottom:16px;}
.input-group label{display:block;font-size:13px;color:#94a3b8;margin-bottom:8px;font-weight:500;}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:20px;}
button{padding:12px 16px;border:1px solid rgba(255,255,255,0.05);border-radius:12px;background:rgba(30,41,59,0.8);color:#e2e8f0;font-size:14px;font-weight:500;cursor:pointer;transition:all .2s cubic-bezier(0.4,0,0.2,1);display:flex;align-items:center;justify-content:center;gap:8px;}
button:hover{background:rgba(51,65,85,0.9);transform:translateY(-1px);color:#fff;border-color:rgba(255,255,255,0.1);}
button:active{transform:scale(0.97);}
button.primary{background:linear-gradient(135deg,#2563eb,#1d4ed8);border:none;color:#fff;box-shadow:0 4px 12px rgba(37,99,235,0.3);}
button.primary:hover{background:linear-gradient(135deg,#3b82f6,#2563eb);box-shadow:0 6px 16px rgba(37,99,235,0.4);border:none;}
.btn-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(130px,1fr));gap:12px;margin-top:16px;}
.btn-icon{font-size:18px;}
footer{color:#64748b;font-size:13px;text-align:center;padding:30px 0;margin-top:auto;border-top:1px solid rgba(255,255,255,0.05);}
.panel{background:rgba(15,23,42,0.3);border-radius:12px;padding:16px;border:1px solid rgba(255,255,255,0.03);margin-top:16px;}
</style>
<script>
let prefLock=false,timeInput=null;
function formatUptime(secs){secs=Number(secs||0);if(secs<0)secs=0;const h=Math.floor(secs/3600),m=Math.floor((secs%3600)/60),s=Math.floor(secs%60);let p=[];if(h)p.push(h+' ч');if(m||h)p.push(m+' м');p.push(s+' с');return p.join(' ');}
async function refresh(){try{let r=await fetch('/data');if(!r.ok)return;let d=await r.json();document.getElementById('ip').innerText=d.ip||'—';document.getElementById('timeNow').innerText=(d.time||'--').replace('T',' ');document.getElementById('uptime').innerText=formatUptime(d.uptime);const badgeIgn=document.getElementById('ignBadge');if(badgeIgn){badgeIgn.innerText=d.ignition?'Зажигание ВКЛ':'Зажигание ВЫКЛ';badgeIgn.className='badge '+(d.ignition?'warn':'err');}const f0=document.getElementById('frames0');if(f0)f0.innerText=d.frames0;const f1=document.getElementById('frames1');if(f1)f1.innerText=d.frames1;const b0=document.getElementById('can0Badge');if(b0){b0.innerText=d.frames0>0?'Car OK':'Car: No data';b0.className='badge '+(d.frames0>0?'ok':'err');}const b1=document.getElementById('can1Badge');if(b1){b1.innerText=d.frames1>0?'Radio OK':'Radio: No data';b1.className='badge '+(d.frames1>0?'ok':'err');}if(!prefLock){const is24=document.getElementById('is24');const isc=document.getElementById('isc');const p260=document.getElementById('p260');const p276=document.getElementById('p276');const lang=document.getElementById('lang');if(is24)is24.checked=!!d.is24;if(isc)isc.checked=!!d.isC;if(lang&&document.activeElement!==lang)lang.value=d.lang||14;if(p260&&document.activeElement!==p260)p260.value=d.p260||500;if(p276&&document.activeElement!==p276)p276.value=d.p276||1000;}if(timeInput&&document.activeElement!==timeInput&&d.time){timeInput.value=d.time.length>=16?d.time.substring(0,16):d.time;}}catch(e){console.error(e);}}
function init(){timeInput=document.getElementById('timeInput');const is24=document.getElementById('is24');const isc=document.getElementById('isc');const p260=document.getElementById('p260');const p276=document.getElementById('p276');const lang=document.getElementById('lang');if(is24)is24.addEventListener('change',()=>updatePref('is24',is24.checked));if(isc)isc.addEventListener('change',()=>updatePref('isc',isc.checked));if(lang)lang.addEventListener('change',()=>updatePref('lang',lang.value));if(p260)p260.addEventListener('change',()=>updatePref('p260',p260.value));if(p276)p276.addEventListener('change',()=>updatePref('p276',p276.value));refresh();setInterval(refresh,1500);}
async function updatePref(name,val){prefLock=true;let strVal = (typeof val === 'boolean') ? (val ? '1' : '0') : encodeURIComponent(val);try{let r=await fetch('/prefs?'+name+'='+strVal);if(!r.ok)throw new Error();}catch(e){alert('Не удалось сохранить настройки');}finally{prefLock=false;}}
function pad2(n){return n.toString().padStart(2,'0');}
function isoLocalNow(){const d=new Date();return d.getFullYear()+'-'+pad2(d.getMonth()+1)+'-'+pad2(d.getDate())+'T'+pad2(d.getHours())+':'+pad2(d.getMinutes())+':'+pad2(d.getSeconds());}
async function syncTime(){const iso=isoLocalNow();try{let r=await fetch('/time?iso='+encodeURIComponent(iso));if(!r.ok)throw new Error();refresh();}catch(e){alert('Не удалось синхронизировать время');}}
async function setTimeFromInput(){if(!timeInput||!timeInput.value){alert('Введите дату и время');return;}let iso=timeInput.value;if(iso.length===16)iso+=':00';try{let r=await fetch('/time?iso='+encodeURIComponent(iso));if(!r.ok)throw new Error();refresh();}catch(e){alert('Не удалось установить время');}}
async function btn(n){await fetch('/btn?n='+n);}
async function sendEmf(c){await fetch('/emf?c='+c);}
document.addEventListener('DOMContentLoaded',init);
</script></head><body>
<header><h1><svg width='24' height='24' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><path d='M14 18V6a2 2 0 0 0-2-2H4a2 2 0 0 0-2 2v11a1 1 0 0 0 1 1h2'/><path d='M15 18H9'/><path d='M19 18h2a1 1 0 0 0 1-1v-3.65a1 1 0 0 0-.22-.624l-3.48-4.35A1 1 0 0 0 17.52 8H14'/><circle cx='17' cy='18' r='2'/><circle cx='7' cy='18' r='2'/></svg> PSA CAN Remote</h1></header>
<main>
<div class='card'>
<h3><svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><circle cx='12' cy='12' r='10'/><polyline points='12 6 12 12 16 14'/></svg> Статус Системы</h3>
<p class='desc'>ESP32-C6 Comfort CAN контроллер. IP: <span id='ip' style='color:#fff;font-weight:500;'>--</span></p>
<div class='metrics'>
<div class='metric'><div class='label'>CAN Скорость</div><div class='value'>125<span class='unit'>kbit/s</span></div></div>
<div class='metric'><div class='label'>Время работы</div><div class='value' id='uptime'>--</div></div>
</div>
<div class='metrics'>
<div class='metric'><div class='label'>Bus 0 (Car)</div><div class='value' id='frames0'>0</div></div>
<div class='metric'><div class='label'>Bus 1 (Radio)</div><div class='value' id='frames1'>0</div></div>
</div>
<div class='badge-container'>
<span class='badge err' id='can0Badge'>Car: No data</span>
<span class='badge err' id='can1Badge'>Radio: No data</span>
<span class='badge err' id='ignBadge'>Зажигание ВЫКЛ</span>
</div>
</div>
<div class='card'>
<h3><svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><polyline points='22 12 18 12 15 21 9 3 6 12 2 12'/></svg> Данные Конфигурации BSI</h3>
<p class='desc'>Текущие широковещательные данные, циклически генерируемые эмулятором для магнитолы SMEG.</p>
<div class='metrics'>
<div class='metric'><div class='label'>Настройки (0x260)</div><div class='value hex' id='bsi260'>--</div></div>
<div class='metric'><div class='label'>Время (0x276)</div><div class='value hex' id='bsi276'>--</div></div>
</div>
</div>
<div class='card'>
<h3><svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><circle cx='12' cy='12' r='3'/><path d='M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z'/></svg> Параметры Автомобиля</h3>
<p class='desc'>Мгновенное применение конфигурации в шину CAN и сохранение.</p>
<div class='control-group'>
<div class='input-group'>
<label for='lang'>Язык системы SMEG</label>
<select id='lang'>
<option value='0'>Французский (Français)</option>
<option value='1'>Английский (English)</option>
<option value='4'>Итальянский (Italiano)</option>
<option value='3'>Испанский (Español)</option>
<option value='2'>Немецкий (Deutsch)</option>
<option value='6'>Голландский (Nederlands)</option>
<option value='5'>Португальский (Português)</option>
<option value='12'>Турецкий (Türkçe)</option>
<option value='13'>Украинский (Українська)</option>
<option value='14'>Русский</option>
<option value='15'>Польский (Polski)</option>
</select></div>
<div class='toggle-wrapper'>
<div class='toggle-label'><span class='toggle-title'>24-часовой Формат</span><span class='toggle-desc'>Использовать 24ч вместо AM/PM</span></div>
<label class='switch'><input type='checkbox' id='is24'/><span class='slider'></span></label>
</div>
<div class='toggle-wrapper'>
<div class='toggle-label'><span class='toggle-title'>Температура в Цельсиях</span><span class='toggle-desc'>Показывать °C вместо °F</span></div>
<label class='switch'><input type='checkbox' id='isc'/><span class='slider'></span></label>
</div>
</div>
</div>
<div class='card'>
<h3><svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><circle cx='12' cy='12' r='10'/><polyline points='12 6 12 12 16 14'/></svg> Частота Трансляции Пакетв (мс)</h3>
<p class='desc'>Увеличьте периоды, если меню SMEG закрывается или мерцает при работе.</p>
<div class='panel'>
<div class='input-group'>
<label for='p260'>Кадр 0x260 (Система/Язык)</label>
<input type='number' id='p260' min='10' max='10000' placeholder='Рекомендуется 500'/>
</div>
<div class='input-group' style='margin-bottom:0;'>
<label for='p276'>Кадр 0x276 (Дата/Время)</label>
<input type='number' id='p276' min='10' max='10000' placeholder='Рекомендуется 1000'/>
</div>
</div>
</div>
<div class='card'>
<h3><svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><rect x='3' y='4' width='18' height='18' rx='2' ry='2'/><line x1='16' y1='2' x2='16' y2='6'/><line x1='8' y1='2' x2='8' y2='6'/><line x1='3' y1='10' x2='21' y2='10'/></svg> Управление Временем</h3>
<div class='metrics' style='margin-bottom:0;'>
<div class='metric'><div class='label'>Встроенные DS3231 (RTC)</div><div class='value' id='timeNow' style='font-size:20px;letter-spacing:1px;'>--</div></div>
</div>
<div class='panel'>
<div class='input-group' style='margin-bottom:0;'>
<label for='timeInput'>Ручная установка</label>
<input type='datetime-local' id='timeInput'/>
</div>
</div>
<div class='controls'>
<button onclick='syncTime()'><svg width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'><path d='M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.59-9.28l5.67-5.67'/></svg> Синхронизировать с ПК</button>
<button class='primary' onclick='setTimeFromInput()'><svg width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'><polyline points='20 6 9 17 4 12'/></svg> Применить Указанное</button>
</div>
</div>
<div class='card'>
<h3><svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><rect x='5' y='2' width='14' height='20' rx='2' ry='2'/><line x1='12' y1='18' x2='12.01' y2='18'/></svg> Макросы Панели</h3>
<p class='desc'>Эмуляция нажатия физических кнопок на мультимедийной панели.</p>
<div class='btn-grid'>
<button onclick='btn(17)'><span class='btn-icon'>⚙️</span> Настройки</button>
<button onclick='btn(18)'><span class='btn-icon'>🗺️</span> Навигация</button>
<button onclick='btn(19)'><span class='btn-icon'>📻</span> Радио</button>
<button onclick='btn(20)'><span class='btn-icon'>📞</span> Телефон</button>
<button onclick='btn(29)'><span class='btn-icon'>📱</span> Приложения</button>
<button onclick='btn(30)'><span class='btn-icon'></span> БК Trip</button>
<button onclick='btn(32)'><span class='btn-icon'>❄️</span> Климат</button>
</div>
</div>
<div class='card'>
<h3>🖥️ Управление EMF-A</h3>
<p class='desc'>Настройка даты/времени старого экрана.</p>
<div class='btn-grid'>
<button onclick='sendEmf(1)' style='background:#1e3a8a;border-color:#3b82f6'>MENU</button>
<button onclick='sendEmf(2)' style='background:#064e3b;border-color:#10b981'>OK</button>
<button onclick='sendEmf(3)' style='background:#7f1d1d;border-color:#ef4444'>ESC</button>
</div>
<div class='btn-grid' style='margin-top:10px;'>
<button onclick='sendEmf(4)'>◀️</button>
<button onclick='sendEmf(5)'>🔼</button>
<button onclick='sendEmf(6)'>🔽</button>
<button onclick='sendEmf(7)'>▶️</button>
</div>
</div>
<div class='card'>
<h3>🛠️ Конфигурация Авто</h3>
<p class='desc'>Включение скрытых функций в меню магнитолы.</p>
<div class='btn-grid'>
<button onclick="location.href='/conf'">🚪 Инверсия Дверей</button>
</div>
</div>
</main>
<footer>ESP32-C6 PSA Gateway • v2.0.0</footer>
</body></html>
)rawliteral";
