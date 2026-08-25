const char INDEX_HTML[] PROGMEM = R"HTMLDOC(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>Parkinson's Hand Tremor · Live Dashboard</title>
<style>
  :root{
    --bg:#0e1320; --panel:#161d2e; --panel2:#1c2436; --line:#28324a;
    --text:#e9eef8; --muted:#8b97b2; --dim:#5f6b85;
    --green:#22c55e; --amber:#f59e0b; --red:#ef4444; --teal:#14b8a6; --accent:#3b82f6;
    --radius:16px;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--text);
       font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;-webkit-text-size-adjust:100%}

  header{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:12px;
         padding:13px 16px;background:rgba(14,19,32,.92);backdrop-filter:blur(8px);
         border-bottom:1px solid var(--line)}
  .brand{display:flex;align-items:center;gap:11px;flex:1;min-width:0}
  .logo{width:34px;height:34px;border-radius:10px;background:var(--accent);
        display:flex;align-items:center;justify-content:center;color:#fff;font-size:19px;flex-shrink:0}
  h1{font-size:16px;margin:0;font-weight:700}
  .sub{font-size:12px;color:var(--muted);margin-top:1px}
  .pill{display:flex;align-items:center;gap:7px;padding:6px 11px;border-radius:999px;
        font-size:13px;font-weight:600;border:1px solid var(--line);background:var(--panel);white-space:nowrap}
  .pill .dot{width:9px;height:9px;border-radius:50%;background:var(--muted)}
  .pill.on{color:var(--green)} .pill.on .dot{background:var(--green);box-shadow:0 0 0 4px rgba(34,197,94,.15)}
  .pill.off{color:var(--red)} .pill.off .dot{background:var(--red)}
  .pill.retry{color:var(--amber)} .pill.retry .dot{background:var(--amber);animation:blink 1s infinite}
  @keyframes blink{50%{opacity:.25}}

  .wrap{padding:16px;max-width:680px;margin:0 auto}
  .modes{display:flex;gap:8px;margin-bottom:14px}
  .mode{flex:1;text-align:center;padding:10px 6px;border-radius:12px;font-size:13px;font-weight:600;
        color:var(--muted);background:var(--panel);border:1px solid var(--line);transition:.25s}
  .mode.active{color:#fff;background:var(--accent);border-color:var(--accent)}

  .panel{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);
         padding:16px;margin-bottom:14px;transition:opacity .3s,border-color .3s,box-shadow .3s;opacity:.5}
  .panel.active{opacity:1;border-color:var(--accent);box-shadow:0 10px 26px rgba(0,0,0,.28)}
  .phead{display:flex;align-items:center;gap:9px;font-size:15px;font-weight:700;margin-bottom:14px}
  .pbar{width:4px;height:17px;border-radius:2px;background:var(--accent)}
  .pbar.amber{background:var(--amber)} .pbar.teal{background:var(--teal)}
  .state-tag{margin-left:auto;font-size:12px;font-weight:600;color:var(--muted);
             padding:4px 10px;border-radius:999px;background:var(--panel2)}
  .state-tag.live{color:#fff;background:var(--accent)}
  .state-tag.live::before{content:"";display:inline-block;width:7px;height:7px;border-radius:50%;
             background:#fff;margin-right:6px;vertical-align:middle;animation:blink 1s infinite}

  .risk-hero{text-align:center;padding:16px;border-radius:12px;background:var(--panel2);margin-bottom:14px;transition:background .3s}
  .risk-hero.g{background:rgba(34,197,94,.10)} .risk-hero.a{background:rgba(245,158,11,.10)} .risk-hero.r{background:rgba(239,68,68,.13)}
  .risk{font-size:26px;font-weight:800;letter-spacing:.03em;color:var(--muted);overflow-wrap:anywhere;line-height:1.15}
  .risk.c-green{color:var(--green)} .risk.c-amber{color:var(--amber)} .risk.c-red{color:var(--red)}

  .label{font-size:12px;color:var(--muted);margin-bottom:6px}
  .grid3{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
  @media(min-width:520px){.grid3{grid-template-columns:repeat(4,1fr)}}
  .metric{background:var(--panel2);border-radius:12px;padding:12px}
  .metric .num{font-size:25px;font-weight:800;line-height:1.05}
  .metric .num small{font-size:13px;color:var(--muted);font-weight:600;margin-left:2px}
  .grade{font-size:32px;font-weight:800;color:var(--muted)}
  .grade.c-green{color:var(--green)} .grade.c-amber{color:var(--amber)} .grade.c-red{color:var(--red)}

  .tag{display:inline-block;padding:4px 10px;border-radius:999px;font-weight:700;font-size:14px}
  .b-green{color:var(--green);background:rgba(34,197,94,.14)}
  .b-amber{color:var(--amber);background:rgba(245,158,11,.14)}
  .b-red{color:var(--red);background:rgba(239,68,68,.14)}
  .b-teal{color:var(--teal);background:rgba(20,184,166,.14)}
  .b-muted{color:var(--muted);background:rgba(139,151,178,.12)}

  .progress{height:8px;background:var(--panel2);border-radius:999px;overflow:hidden;margin-top:14px}
  .progress-fill{height:100%;width:0;background:var(--amber);border-radius:999px;transition:width .3s}
  canvas{width:100%;height:150px;display:block;background:#0a0e18;border-radius:12px;margin-top:6px}

  .bar{display:flex;gap:8px;margin-top:2px}
  input{flex:1;background:var(--panel);border:1px solid var(--line);color:var(--text);
        border-radius:10px;padding:10px 12px;font-size:14px;min-width:0}
  button{background:var(--accent);color:#fff;border:0;border-radius:10px;padding:10px 16px;
         font-size:14px;font-weight:700;cursor:pointer;white-space:nowrap}
  .foot{color:var(--dim);font-size:12px;margin-top:12px;text-align:center;line-height:1.7}
  .tools{display:flex;gap:8px;margin-bottom:14px;flex-wrap:wrap}
  .tool{background:var(--panel);border:1px solid var(--line);color:var(--text);border-radius:10px;padding:9px 14px;font-size:13px;font-weight:600;cursor:pointer}
  .tool.rec{color:var(--red);border-color:var(--red)}
  .replay-bar{display:flex;align-items:center;gap:10px;justify-content:space-between;background:rgba(245,158,11,.12);border:1px solid var(--amber);color:var(--amber);border-radius:10px;padding:10px 14px;margin-bottom:14px;font-size:13px;font-weight:700}
  .mini{background:var(--amber);color:#0e1320;border:0;border-radius:8px;padding:6px 12px;font-weight:700;cursor:pointer}
  .panel.lit{opacity:1}
  .hist{display:flex;flex-direction:column;gap:6px}
  .hist-row{display:flex;align-items:center;gap:10px;font-size:13px;color:var(--muted);background:var(--panel2);border-radius:9px;padding:8px 12px}
  .hist-row .t{color:var(--text);font-variant-numeric:tabular-nums}
  .hist-row .sp{margin-left:auto}
  .hist-empty{color:var(--dim);font-size:13px;text-align:center;padding:10px}
  #spec{height:120px}
  .specnote{font-size:12px;font-weight:600;margin-left:8px}
  .patient{display:flex;align-items:center;gap:10px;background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:10px 14px;margin-bottom:14px}
  .patient input{background:var(--panel2)}
  .pbar.purple{background:#8b7cf6}
  .steps{display:flex;gap:8px;margin-bottom:14px}
  .step{flex:1;text-align:center;padding:8px 4px;border-radius:10px;font-size:12px;font-weight:600;color:var(--muted);background:var(--panel2);border:1px solid var(--line)}
  .step.on{color:#fff;background:var(--accent);border-color:var(--accent)}
  .step.done{color:var(--green);border-color:var(--green)}
  .sub2{font-size:12px;color:var(--muted)}
  .cmp{width:100%;border-collapse:collapse;font-size:13px;margin-top:6px}
  .cmp th,.cmp td{padding:8px 10px;border-bottom:1px solid var(--line);text-align:left}
  .cmp th{color:var(--muted);font-weight:600}
  .cmp td:first-child{color:var(--muted)}
  .better{color:var(--green);font-weight:700}
  .worse{color:var(--red);font-weight:700}
</style>
</head>
<body>

<header>
  <div class="brand">
    <div class="logo">~</div>
    <div>
      <h1>Parkinson's Hand Tremor</h1>
      <div class="sub">Live Dashboard</div>
    </div>
  </div>
  <div id="pill" class="pill off"><span class="dot"></span><span id="connText">Disconnected</span></div>
</header>

<div class="wrap">

  <!-- Patient info (clinic use) -->
  <div class="patient">
    <span class="label">Patient</span>
    <input id="ptName" placeholder="Name / MRN (optional)">
  </div>

  <!-- Mode indicator (read-only: device mode is switched by gesture) -->
  <div class="modes">
    <div class="mode" data-mode="tremor">Tremor</div>
    <div class="mode" data-mode="tap">Finger Tap</div>
    <div class="mode" data-mode="monitor">Monitor</div>
    <div class="mode" data-mode="guided">Guided Exam</div>
  </div>

  <div id="replayBar" class="replay-bar" style="display:none">
    <span>Replay mode (from file, not live)</span>
    <button id="btnLive" class="mini">Back to Live</button>
  </div>


  <!-- Section 1: Tremor measurement -->
  <section id="sec-tremor" class="panel">
    <div class="phead"><span class="pbar"></span>Tremor Measurement<span id="state" class="state-tag">—</span></div>
    <div id="riskHero" class="risk-hero">
      <div class="label">Risk Assessment</div>
      <div id="risk" class="risk">—</div>
    </div>
    <div class="grid3">
      <div class="metric"><div class="label">Peak Freq</div><div class="num"><span id="freq">–</span><small>Hz</small></div></div>
      <div class="metric"><div class="label">Amplitude</div><div class="num"><span id="mg">–</span><small>mg</small></div></div>
      <div class="metric"><div class="label">Severity</div><div><span id="sev" class="tag b-muted">—</span></div></div>
      <div class="metric"><div class="label">Regularity</div><div class="num"><span id="br">–</span></div></div>
    </div>
    <div class="label" style="margin-top:14px">Waveform</div>
    <canvas id="wave"></canvas>
    <div class="label" style="margin-top:14px">Spectrum (FFT)<span id="specNote" class="specnote"></span></div>
    <canvas id="spec"></canvas>
  </section>

  <!-- Section 2: Finger tap test -->
  <section id="sec-tap" class="panel">
    <div class="phead"><span class="pbar amber"></span>Finger Tap Test<span id="tapState" class="state-tag">—</span></div>
    <div class="grid3">
      <div class="metric"><div class="label">Bradykinesia Grade</div><div id="tapGrade" class="grade">—</div></div>
      <div class="metric"><div class="label">Taps</div><div class="num"><span id="tapCount">–</span><small>/10</small></div></div>
      <div class="metric"><div class="label">Mean Interval</div><div class="num"><span id="tapMean">–</span><small>ms</small></div></div>
      <div class="metric"><div class="label">Rhythm CV</div><div class="num"><span id="tapCv">–</span></div></div>
    </div>
    <div class="progress"><div id="tapBar" class="progress-fill"></div></div>
  </section>

  <!-- Section 3: Continuous monitor -->
  <section id="sec-monitor" class="panel">
    <div class="phead"><span class="pbar teal"></span>Continuous Monitor<span id="monState" class="state-tag">—</span></div>
    <div class="grid3">
      <div class="metric"><div class="label">Current State</div><div><span id="monActive" class="tag b-muted">—</span></div></div>
      <div class="metric"><div class="label">Episodes</div><div class="num"><span id="monEp">–</span></div></div>
      <div class="metric"><div class="label">Elapsed</div><div class="num"><span id="monTime">–</span><small>s</small></div></div>
      <div class="metric"><div class="label">Current Freq</div><div class="num"><span id="monFreq">–</span><small>Hz</small></div></div>
    </div>
    <div class="label" style="margin-top:14px">Waveform</div>
    <canvas id="waveMon"></canvas>
    <div class="label" style="margin-top:14px">Spectrum (FFT)<span id="specNoteMon" class="specnote"></span></div>
    <canvas id="specMon"></canvas>
  </section>

  <!-- Section 4: Guided exam -->
  <section id="sec-guided" class="panel">
    <div class="phead"><span class="pbar purple"></span>Guided Exam<span id="gdState" class="state-tag">—</span></div>
    <div class="steps">
      <div id="stRest" class="step">① Rest Tremor</div>
      <div id="stTap" class="step">② Finger Tap</div>
      <div id="stPosture" class="step">③ Posture</div>
    </div>
    <div class="grid3">
      <div class="metric"><div class="label">Rest · Freq</div><div class="num"><span id="gdRestFreq">–</span><small>Hz</small></div></div>
      <div class="metric"><div class="label">Rest · Amp</div><div class="num"><span id="gdRestMg">–</span><small>mg</small></div></div>
      <div class="metric"><div class="label">Tap Grade</div><div id="gdTapGrade" class="grade" style="font-size:22px">—</div></div>
      <div class="metric"><div class="label">Posture · Amp</div><div class="num"><span id="gdPostMg">–</span><small>mg</small></div></div>
    </div>
    <div style="margin-top:14px;padding:14px 12px;background:var(--panel2);border-radius:12px;text-align:center">
      <div class="label" style="margin-bottom:2px">Composite Score CMPI</div>
      <div style="font-size:42px;font-weight:800;line-height:1.05"><span id="gdCmpi">–</span><small style="font-size:15px;color:var(--muted);font-weight:600">/100</small></div>
      <div class="sub2" style="margin-top:6px">XGBoost</div>
    </div>
    <div style="margin-top:12px"><span id="gdRisk" class="badge b-muted">—</span></div>
  </section>

  <!-- Pre/post medication comparison (clinic use, based on guided exam) -->
  <section class="panel lit">
    <div class="phead"><span class="pbar"></span>Pre/Post Medication Comparison</div>
    <div class="tools">
      <button id="btnMedBefore" class="tool">Record as Pre-Med</button>
      <button id="btnMedAfter" class="tool">Record as Post-Med</button>
      <button id="btnMedExport" class="tool">Export Report</button>
      <button id="btnMedClear" class="tool">Clear</button>
    </div>
    <div id="cmpBox"></div>
  </section>

  <!-- Measurement history -->
  <section class="panel lit">
    <div class="phead"><span class="pbar"></span>Measurement History</div>
    <div id="histList" class="hist"></div>
  </section>

  <!-- Record / replay (demo fallback) -->
  <div class="tools">
    <button id="btnRec" class="tool">Record</button>
    <label class="tool" for="fileReplay">Load Replay</label>
    <input id="fileReplay" type="file" accept="application/json" hidden>
    <button id="btnCsv" class="tool">Export CSV</button>
  </div>

  <!-- Connection settings (auto-connect; normally untouched) -->
  <div class="bar">
    <input id="url" placeholder="ws://...">
    <button id="btn">Connect</button>
  </div>
  <div class="foot">Source: <span id="src">—</span> · Last update: <span id="ts">—</span></div>

</div>

<script>
/* ============================================================
   Live dashboard — front-end logic
   Data flow: ESP32 (or mock server) --WebSocket--> page --update UI by type
   Message types (routed by "type"):
     wave      : {type,t,amp}                                 live waveform point (monitor only)
     wave_full : {type,fs,amp:[...]}                          full waveform sent once after measurement
     result    : {type,t,state,freq,mg,band_ratio,severity,risk} tremor result
     tap     : {type,t,state,count,n,mean_ms,cv,grade}        finger tap test
     monitor : {type,t,active,episodes,elapsed_s,freq,mg}     continuous monitor
   ============================================================ */
const $ = id => document.getElementById(id);
let ws = null, wantConnected = false, retryTimer = null;

/* ---- Connection light: 'on' connected / 'retry' connecting / 'off' disconnected ---- */
function setConn(status){
  $('pill').className = 'pill ' + status;
  $('connText').textContent = status==='on' ? 'Connected' : status==='retry' ? 'Connecting…' : 'Disconnected';
  $('btn').textContent = wantConnected ? 'Disconnect' : 'Connect';
}

/* ---- Mode highlight (read-only: device mode switched by gesture, page just reflects it) ---- */
function setMode(m){
  document.querySelectorAll('.mode').forEach(el => el.classList.toggle('active', el.dataset.mode === m));
  for(const [id, mode] of [['sec-tremor','tremor'],['sec-tap','tap'],['sec-monitor','monitor'],['sec-guided','guided']]){
    const el = $(id), on = (mode === m);
    el.style.display = on ? 'block' : 'none';   // show only the current mode's dashboard
    el.classList.toggle('active', on);
  }
}

/* ---- Waveform: self-drawn canvas, no CDN, works offline (draws to the visible mode canvas) ---- */
const buf = []; const MAXPTS = 300;
function pushWave(v){ buf.push(v); if(buf.length > MAXPTS) buf.shift(); }
function drawWaveTo(cv){
  const ctx = cv.getContext('2d');
  const w = cv.width  = cv.clientWidth  * devicePixelRatio;
  const h = cv.height = cv.clientHeight * devicePixelRatio;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = '#1c2740'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, h/2); ctx.lineTo(w, h/2); ctx.stroke();
  if(buf.length > 1){
    let mx = 0.02; for(const v of buf) mx = Math.max(mx, Math.abs(v));
    ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 2; ctx.beginPath();
    for(let i = 0; i < buf.length; i++){
      const x = i / Math.max(1, buf.length - 1) * w, y = h/2 - (buf[i]/mx) * (h*0.45);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
    ctx.stroke();
  }
}
function draw(){
  for(const id of ['wave','waveMon']){ const c = $(id); if(c && c.offsetParent !== null) drawWaveTo(c); }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

/* ---- String -> color / label maps ---- */
const riskColor = r => r==='PD-RISK'?'c-red' : r==='PD-LIKE'?'c-amber' : r==='NO-PD-SIGNAL'?'c-green' : '';
const riskTint  = r => r==='PD-RISK'?'r'     : r==='PD-LIKE'?'a'       : r==='NO-PD-SIGNAL'?'g'        : '';
const sevClass  = s => (s==='SEVERE'||s==='MODERATE')?'b-red' : s==='MILD'?'b-amber' : s==='MOTION'?'b-muted' : 'b-green';
const stateText = s => ({ready:'Ready', settling:'Settling', measuring:'Measuring', done:'Done ✓'}[s] || s || '—');
const gradeColor= g => g==='G0'?'c-green' : (g==='G1'||g==='G2')?'c-amber' : 'c-red';
const motorText = mp => ({
  STRONG_RHYTHMIC_TREMOR:{txt:'Strong Rhythmic Tremor',c:'c-red',tint:'r'},
  RHYTHMIC_TREMOR:{txt:'Rhythmic Tremor',c:'c-amber',tint:'a'},
  NON_RHYTHMIC_MOTION:{txt:'Non-Rhythmic Motion',c:'',tint:''},
  NO_CHARACTERISTIC_TREMOR:{txt:'No Characteristic Tremor',c:'c-green',tint:'g'}
}[mp] || {txt:mp,c:'',tint:''});
const signalText = sl => ({
  HIGH:{txt:'Signal High',c:'b-red'}, MEDIUM:{txt:'Signal Medium',c:'b-amber'},
  LOW:{txt:'Signal Low',c:'b-muted'}, BELOW_THRESHOLD:{txt:'Below Threshold',c:'b-green'}
}[sl] || {txt:sl,c:'b-muted'});

/* ---- On message: route by type ---- */
function onMsg(raw){
  let d; try{ d = JSON.parse(raw); }catch(e){ return; }
  recordRaw(raw);
  $('ts').textContent = new Date().toLocaleTimeString();

  if(d.type === 'mode'){ if(d.mode==='guided'){ guidedActive=true; guidedLive={}; } else guidedActive=false; setMode(d.mode); return; }   // device picked a mode, switch page

  if(d.type === 'wave'){ const a=Number(d.amp)||0; pushWave(a); pushSpec(a, d.t); sessionWave.push({t:(d.t!=null?d.t:Date.now()), amp:a}); if(sessionWave.length>18000) sessionWave.shift(); return; }

  if(d.type === 'wave_full'){                                  // measurement done: full waveform in one message, draw the whole thing
    const arr = Array.isArray(d.amp) ? d.amp.map(Number) : [];
    if(!arr.length) return;
    const fs = Number(d.fs) || 20;                             // per-point effective sample rate (Hz)
    const dt = 1000 / fs;                                      // per-point interval (ms)
    const t0 = Date.now();
    buf.length = 0;                                            // waveform canvas: replace with the full trace
    for(const a of arr){ buf.push(a); if(buf.length > MAXPTS) buf.shift(); }
    waveAmp.length = 0; waveTs.length = 0;                     // spectrum: rebuild from the full trace (last NFFT points)
    arr.forEach((a,i)=> pushSpec(a, t0 + i*dt));
    sessionWave.length = 0;                                    // CSV export: rebuild the full trace
    arr.forEach((a,i)=> sessionWave.push({t: t0 + Math.round(i*dt), amp: a}));
    return;
  }

  if(d.type === 'result'){
    if(guidedActive){   // guided exam "① Rest Tremor" stage: fill fields in place, light the step, don't switch page
      guidedLive.rest = { freq:num(d.freq), mg:num(d.mg), band_ratio:num(d.band_ratio), risk:d.risk, motor_pattern:d.motor_pattern };
      if(d.freq != null) $('gdRestFreq').textContent = fmt1(d.freq);
      if(d.mg   != null) $('gdRestMg').textContent   = fmtInt(d.mg);
      if(d.classification_suppressed){ const e=$('gdRisk'); e.textContent='Low quality — classification blocked'; e.className='badge b-red'; }
      else if(d.risk != null){ const e=$('gdRisk'); e.textContent=d.risk; e.className='badge '+riskBadge(d.risk); }
      else if(d.motor_pattern != null){ const m=motorText(d.motor_pattern); const e=$('gdRisk'); e.textContent=m.txt;
        e.className='badge '+(m.tint==='r'?'b-red':m.tint==='a'?'b-amber':m.tint==='g'?'b-green':'b-muted'); }
      $('stRest').className='step done';
      { const e=$('gdState'); e.textContent='① Rest done →'; e.classList.remove('live'); }
      return;
    }
    setMode('tremor');
    if(d.state === 'done' && lastTremorState !== 'done') logHistory(d);
    lastTremorState = d.state;
    if(d.state != null){ const e=$('state'); e.textContent = stateText(d.state); e.classList.toggle('live', d.state==='measuring'); }
    if(d.freq != null){ $('freq').textContent = Number(d.freq).toFixed(1); peakHz = Number(d.freq); updateSpecNote(); }
    if(d.mg != null)         $('mg').textContent   = Math.round(d.mg);
    if(d.band_ratio != null) $('br').textContent   = Number(d.band_ratio).toFixed(2);
    if(d.classification_suppressed){
      const e=$('sev'); e.textContent='Repeat test'; e.className='tag b-red';
      $('risk').textContent='Low quality — classification blocked'; $('risk').className='risk'; $('riskHero').className='risk-hero';
      if(d.compute_source != null) $('state').textContent='Done · '+d.compute_source;
      return;
    }
    if(d.severity != null){ const e=$('sev'); e.textContent=d.severity; e.className='tag '+sevClass(d.severity); }
    if(d.risk != null){ $('risk').textContent=d.risk; $('risk').className='risk '+riskColor(d.risk);
                        $('riskHero').className='risk-hero '+riskTint(d.risk); }
    if(d.signal_level != null){ const s=signalText(d.signal_level); const e=$('sev'); e.textContent=s.txt; e.className='tag '+s.c; }
    if(d.motor_pattern != null){ const m=motorText(d.motor_pattern); $('risk').textContent=m.txt; $('risk').className='risk '+m.c; $('riskHero').className='risk-hero '+m.tint; }
    return;
  }

  if(d.type === 'tap'){
    if(guidedActive){   // guided exam "② Finger Tap" stage
      guidedLive.tap = { grade:d.grade, mean_ms:num(d.mean_ms), cv:num(d.cv), count:num(d.count) };
      if(d.grade != null){ $('gdTapGrade').textContent=d.grade; $('gdTapGrade').className='grade '+gradeColor(d.grade); }
      const done = d.state==='done';
      $('stTap').className='step '+(done?'done':'on');
      { const e=$('gdState'); e.textContent=done?'② Tap done →':'② Tapping…'; e.classList.toggle('live', !done); }
      return;
    }
    setMode('tap');
    const tapping = d.state==='tapping';
    { const e=$('tapState'); e.textContent = tapping?'Tapping':'Done ✓'; e.classList.toggle('live', tapping); }
    if(d.grade != null){ $('tapGrade').textContent=d.grade; $('tapGrade').className='grade '+gradeColor(d.grade); }
    if(d.count != null) $('tapCount').textContent = d.count;
    if(d.mean_ms != null) $('tapMean').textContent = Math.round(d.mean_ms);
    if(d.cv != null) $('tapCv').textContent = Number(d.cv).toFixed(2);
    const n = d.n || 10; if(d.count != null) $('tapBar').style.width = (Math.min(d.count,n)/n*100) + '%';
    return;
  }

  if(d.type === 'monitor'){
    if(guidedActive){   // guided exam "③ Posture" stage
      guidedLive.posture = { mg:num(d.mg), freq:num(d.freq) };
      if(d.mg != null) $('gdPostMg').textContent = fmtInt(d.mg);
      $('stPosture').className='step on';
      { const e=$('gdState'); e.textContent='③ Measuring posture…'; e.classList.add('live'); }
      return;
    }
    setMode('monitor');
    { const e=$('monState'); e.textContent='Monitoring'; e.classList.add('live'); }
    if(d.active != null){ const e=$('monActive'); e.textContent = d.active?'Tremor detected':'Normal';
                          e.className = 'tag ' + (d.active?'b-red':'b-green'); }
    if(d.episodes != null)  $('monEp').textContent   = d.episodes;
    if(d.elapsed_s != null) $('monTime').textContent = d.elapsed_s;
    if(d.freq != null){ $('monFreq').textContent = Number(d.freq).toFixed(1); peakHz = Number(d.freq); updateSpecNote(); }
    return;
  }

  if(d.type === 'guided'){
    guidedActive = true;
    setMode('guided');
    const map = {rest:'stRest', tap:'stTap', posture:'stPosture'};
    if(d.state && map[d.state]){ for(const k in map) $(map[k]).className = 'step' + (d.state===k ? ' on' : ''); }
    { const done = d.state==='done'; const e=$('gdState');
      e.textContent = done ? 'Done ✓' : (d.state==='rest'?'Measuring rest tremor…':d.state==='tap'?'Tapping…':d.state==='posture'?'Measuring posture…':'—');
      e.classList.toggle('live', !done && !!d.state); }
    if(d.state==='done'){
      d.rest = d.rest || guidedLive.rest; d.tap = d.tap || guidedLive.tap; d.posture = d.posture || guidedLive.posture;  // merge in per-stage live values (firmware "done" only carries scores)
      for(const id of ['stRest','stTap','stPosture']) $(id).className='step done';
      if(d.rest){ $('gdRestFreq').textContent=fmt1(d.rest.freq); $('gdRestMg').textContent=fmtInt(d.rest.mg);
        if(d.rest.risk){ const e=$('gdRisk'); e.textContent=d.rest.risk; e.className='badge '+riskBadge(d.rest.risk); } }
      if(d.tap && d.tap.grade){ $('gdTapGrade').textContent=d.tap.grade; $('gdTapGrade').className='grade '+gradeColor(d.tap.grade); }
      if(d.posture){ $('gdPostMg').textContent=fmtInt(d.posture.mg); }
      { const sup = d.suppressed || d.cmpi==null || Number(d.cmpi) < 0;   // composite score CMPI (no quality tag shown)
        $('gdCmpi').textContent = sup ? '—' : Math.round(d.cmpi); }
      lastGuided = d;
      guidedLive = {};   // clear after this run, ready for next (e.g. post-med)
    }
    return;
  }
}

/* ---- Connection: auto-connect + auto-reconnect on drop ---- */
function connect(){
  wantConnected = true;
  const url = $('url').value.trim();
  $('src').textContent = url;
  setConn('retry');
  try{ ws = new WebSocket(url); }
  catch(e){ scheduleRetry(); return; }
  ws.onopen    = () => setConn('on');
  ws.onmessage = ev => onMsg(ev.data);
  ws.onclose   = () => { ws = null; setConn(wantConnected ? 'retry' : 'off'); scheduleRetry(); };
  ws.onerror   = () => {};
}
function scheduleRetry(){
  if(!wantConnected) return;                 // don't reconnect if the user disconnected on purpose
  clearTimeout(retryTimer);
  retryTimer = setTimeout(connect, 2000);    // retry after 2 s
}
function disconnect(){
  wantConnected = false;
  clearTimeout(retryTimer);
  if(ws){ ws.close(); ws = null; }
  setConn('off');
}
$('btn').onclick = () => wantConnected ? disconnect() : connect();

/* ---- Measurement history (one entry per completed measurement) ---- */
const histLog = []; let lastTremorState = null;
const riskBadge = r => r==='PD-RISK'?'b-red' : r==='PD-LIKE'?'b-amber' : r==='NO-PD-SIGNAL'?'b-green' : 'b-muted';
function logHistory(d){
  histLog.unshift({ time:new Date().toLocaleTimeString(), risk:d.risk||'—',
                    freq:(d.freq!=null?Number(d.freq).toFixed(1):'–'), mg:(d.mg!=null?Math.round(d.mg):'–') });
  if(histLog.length > 10) histLog.pop();
  renderHistory();
}
function renderHistory(){
  const box = $('histList');
  if(!histLog.length){ box.innerHTML = '<div class="hist-empty">No records yet — a completed measurement will appear here</div>'; return; }
  box.innerHTML = histLog.map(h =>
    '<div class="hist-row"><span class="t">'+h.time+'</span>'+
    '<span class="tag '+riskBadge(h.risk)+'">'+h.risk+'</span>'+
    '<span class="sp">'+h.freq+' Hz</span><span>'+h.mg+' mg</span></div>').join('');
}

/* ---- Record / replay / export (demo fallback, works offline) ---- */
let recording=false, recorded=[], recStart=0, replaying=false, replayTimers=[];
function recordRaw(raw){ if(recording) recorded.push({dt: performance.now()-recStart, raw:raw}); }
function toggleRecord(){
  if(replaying) return;
  if(!recording){
    recording=true; recorded=[]; recStart=performance.now();
    $('btnRec').textContent='Stop & Download'; $('btnRec').classList.add('rec');
  }else{
    recording=false; $('btnRec').textContent='Record'; $('btnRec').classList.remove('rec');
    if(recorded.length) downloadJSON(recorded); else alert('No data recorded');
  }
}
function downloadJSON(data){
  const blob=new Blob([JSON.stringify(data)],{type:'application/json'});
  const a=document.createElement('a');
  a.href=URL.createObjectURL(blob); a.download='tremor_recording_'+Date.now()+'.json';
  a.click(); setTimeout(()=>URL.revokeObjectURL(a.href), 1000);
}
function loadReplay(file){
  const rd=new FileReader();
  rd.onload=()=>{ try{ startReplay(JSON.parse(rd.result)); }catch(e){ alert('Invalid file format: '+e.message); } };
  rd.readAsText(file);
}
function startReplay(data){
  if(!Array.isArray(data) || !data.length){ alert('This file has no replayable data'); return; }
  disconnect(); stopReplay();
  replaying=true; $('replayBar').style.display='flex';
  for(const it of data) replayTimers.push(setTimeout(()=>onMsg(it.raw), it.dt));
  replayTimers.push(setTimeout(stopReplay, data[data.length-1].dt + 300));
}
function stopReplay(){ replayTimers.forEach(clearTimeout); replayTimers=[]; replaying=false; $('replayBar').style.display='none'; }
function backToLive(){ stopReplay(); connect(); }

$('btnRec').onclick = toggleRecord;
$('btnLive').onclick = backToLive;
$('fileReplay').onchange = e => { if(e.target.files[0]) loadReplay(e.target.files[0]); e.target.value=''; };
renderHistory();

/* ---- Live spectrum (FFT): draw frequency, shade the 3–7Hz PD band + mark device peak ---- */
/* Spectrum draws to the visible mode canvas (spec or specMon) */
const waveAmp = [], waveTs = []; const NFFT = 128; let peakHz = null;
const sessionWave = [];                       // for CSV export (t_ms, amp)
function pushSpec(amp, t){
  waveAmp.push(amp); if(waveAmp.length > NFFT) waveAmp.shift();
  waveTs.push(t!=null?t:performance.now()); if(waveTs.length > NFFT) waveTs.shift();
}
function estimateFs(){
  if(waveTs.length < 8) return 30;
  const span = waveTs[waveTs.length-1] - waveTs[0];
  return span > 0 ? (waveTs.length-1)*1000/span : 30;
}
function fft(re, im){
  const n = re.length;
  for(let i=1, j=0; i<n; i++){
    let bit = n>>1;
    for(; j & bit; bit>>=1) j ^= bit;
    j ^= bit;
    if(i < j){ const tr=re[i]; re[i]=re[j]; re[j]=tr; const ti=im[i]; im[i]=im[j]; im[j]=ti; }
  }
  for(let len=2; len<=n; len<<=1){
    const ang = -2*Math.PI/len, wr = Math.cos(ang), wi = Math.sin(ang);
    for(let i=0; i<n; i+=len){
      let cr=1, ci=0;
      for(let k=0; k<len/2; k++){
        const ur=re[i+k], ui=im[i+k];
        const vr=re[i+k+len/2]*cr - im[i+k+len/2]*ci;
        const vi=re[i+k+len/2]*ci + im[i+k+len/2]*cr;
        re[i+k]=ur+vr; im[i+k]=ui+vi; re[i+k+len/2]=ur-vr; im[i+k+len/2]=ui-vi;
        const ncr=cr*wr-ci*wi; ci=cr*wi+ci*wr; cr=ncr;
      }
    }
  }
}
function drawSpecTo(cv){
  const sctx = cv.getContext('2d');
  const w = cv.width = cv.clientWidth * devicePixelRatio;
  const h = cv.height = cv.clientHeight * devicePixelRatio;
  sctx.clearRect(0,0,w,h);
  const FMAX = 12, fs = estimateFs(), x = f => f/FMAX*w;
  sctx.fillStyle = 'rgba(59,130,246,.13)'; sctx.fillRect(x(3), 0, x(7)-x(3), h);
  sctx.fillStyle = '#5f6b85'; sctx.font = (10*devicePixelRatio)+'px sans-serif'; sctx.textAlign = 'center';
  for(let f=0; f<=FMAX; f+=2){ sctx.fillText(String(f), x(f), h-4); }
  if(waveAmp.length >= NFFT){
    let mean=0; for(const v of waveAmp) mean += v; mean /= NFFT;
    const re = new Array(NFFT), im = new Array(NFFT).fill(0);
    for(let i=0;i<NFFT;i++){ const win = 0.5 - 0.5*Math.cos(2*Math.PI*i/(NFFT-1)); re[i] = (waveAmp[i]-mean)*win; }
    fft(re, im);
    const half = NFFT/2; let mx = 1e-9; const mag = [];
    for(let k=0;k<half;k++){ const m = Math.hypot(re[k], im[k]); mag.push(m); if(m>mx) mx=m; }
    sctx.strokeStyle = '#3b82f6'; sctx.lineWidth = 2*devicePixelRatio; sctx.beginPath(); let started=false;
    for(let k=0;k<half;k++){ const f = k*fs/NFFT; if(f>FMAX) break;
      const px = x(f), py = h-14 - (mag[k]/mx)*(h-24);
      started ? sctx.lineTo(px,py) : sctx.moveTo(px,py); started=true; }
    sctx.stroke();
  }
  if(peakHz!=null && peakHz>0 && peakHz<=FMAX){
    const px = x(peakHz);
    sctx.strokeStyle = '#22c55e'; sctx.lineWidth = 2*devicePixelRatio;
    sctx.setLineDash([5*devicePixelRatio,4*devicePixelRatio]); sctx.beginPath();
    sctx.moveTo(px,0); sctx.lineTo(px,h-14); sctx.stroke(); sctx.setLineDash([]);
  }
}
function drawSpec(){
  for(const id of ['spec','specMon']){ const c = $(id); if(c && c.offsetParent !== null) drawSpecTo(c); }
  requestAnimationFrame(drawSpec);
}
requestAnimationFrame(drawSpec);
function updateSpecNote(){
  const inband = (peakHz>=3 && peakHz<=7);
  const txt = (peakHz==null || peakHz<=0) ? '' : 'Peak '+peakHz.toFixed(1)+' Hz'+(inband?' (within 3–7Hz PD band)':' (out of band)');
  for(const id of ['specNote','specNoteMon']){
    const el = $(id); if(!el) continue;
    el.textContent = txt; el.style.color = inband ? 'var(--red)' : 'var(--muted)';
  }
}

/* ---- Export CSV (raw waveform signal, for Python analysis) ---- */
function exportCSV(){
  if(!sessionWave.length){ alert('No waveform data yet — run a measurement first'); return; }
  let csv = 't_ms,amp\n';
  for(const s of sessionWave) csv += s.t + ',' + s.amp + '\n';
  const blob = new Blob([csv], {type:'text/csv'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = 'tremor_wave_'+Date.now()+'.csv';
  a.click(); setTimeout(()=>URL.revokeObjectURL(a.href), 1000);
}
$('btnCsv').onclick = exportCSV;

/* ---- Guided exam + pre/post medication comparison (clinic use) ---- */
const fmt1  = v => v!=null ? Number(v).toFixed(1) : '–';
const fmtInt= v => v!=null ? Math.round(v) : '–';
let lastGuided = null, medBefore = null, medAfter = null, guidedActive = false, guidedLive = {};

function snapMed(which){
  if(!lastGuided){ alert('No guided exam result yet — run a guided exam first'); return; }
  const snap = { patient: $('ptName').value.trim(), time: new Date().toLocaleString(), g: lastGuided };
  if(which==='before') medBefore = snap; else medAfter = snap;
  renderCompare();
}
function num(x){ return (x==null || isNaN(x)) ? null : Number(x); }
function cmpRow(label, b, a, unit, lowerBetter){
  const bv=num(b), av=num(a); const dp = unit==='Hz' ? 1 : 0;
  let chg='—', cls='';
  if(bv!=null && av!=null){
    const d = av-bv;
    if(Math.abs(d) < (dp?0.05:0.5)){ chg='no change'; }
    else{
      const pct = bv!==0 ? Math.round(Math.abs(d)/Math.abs(bv)*100) : null;
      const improved = lowerBetter ? d<0 : d>0;
      chg = (d<0?'↓':'↑') + Math.abs(d).toFixed(dp) + (unit&&unit!=='Hz'?unit:(unit==='Hz'?'Hz':'')) + (pct!=null?(' ('+pct+'%)'):'');
      cls = improved ? 'better' : 'worse';
    }
  }
  const fb = bv!=null ? bv.toFixed(dp) : '–';
  const fa = av!=null ? av.toFixed(dp) : '–';
  return '<tr><td>'+label+'</td><td>'+fb+'</td><td>'+fa+'</td><td class="'+cls+'">'+chg+'</td></tr>';
}
function renderCompare(){
  const box=$('cmpBox');
  const name = $('ptName').value.trim() || (medBefore&&medBefore.patient) || (medAfter&&medAfter.patient) || '';
  if(!medBefore && !medAfter){
    box.innerHTML='<div class="hist-empty">Run a guided exam → click "Record as Pre-Med"; after medication run again → click "Record as Post-Med" to see the comparison.</div>';
    return;
  }
  const b = medBefore?medBefore.g:null, a = medAfter?medAfter.g:null;
  const gr = (o,k,f)=> (o&&o[k]!=null) ? f(o[k]) : null;
  let html = (name?('<div class="sub2">Patient: '+name+'</div>'):'') +
    '<table class="cmp"><tr><th>Metric</th><th>Pre-Med</th><th>Post-Med</th><th>Change</th></tr>';
  html += cmpRow('Composite CMPI', gr(b,'cmpi',v=>v>=0?v:null), gr(a,'cmpi',v=>v>=0?v:null), '', true);
  html += cmpRow('Rest Tremor Amp (mg)', gr(b,'rest',r=>r.mg), gr(a,'rest',r=>r.mg), '', true);
  html += cmpRow('Rest Tremor Freq (Hz)', gr(b,'rest',r=>r.freq), gr(a,'rest',r=>r.freq), 'Hz', false);
  html += cmpRow('Tap Mean Interval (ms)', gr(b,'tap',x=>x.mean_ms), gr(a,'tap',x=>x.mean_ms), '', true);
  html += '<tr><td>Tap Grade</td><td>'+(gr(b,'tap',x=>x.grade)||'–')+'</td><td>'+(gr(a,'tap',x=>x.grade)||'–')+'</td><td></td></tr>';
  html += cmpRow('Posture Amp (mg)', gr(b,'posture',p=>p.mg), gr(a,'posture',p=>p.mg), '', true);
  html += '</table>';
  html += '<div class="sub2" style="margin-top:8px">Pre: '+(medBefore?medBefore.time:'—')+'   Post: '+(medAfter?medAfter.time:'—')+'</div>';
  box.innerHTML = html;
}
function exportMed(){
  if(!medBefore && !medAfter){ alert('No records to export'); return; }
  const rec = { patient: $('ptName').value.trim(), before: medBefore, after: medAfter, exported: new Date().toISOString() };
  const blob=new Blob([JSON.stringify(rec,null,2)],{type:'application/json'});
  const a=document.createElement('a'); a.href=URL.createObjectURL(blob);
  a.download='patient_'+(rec.patient||'anon')+'_'+Date.now()+'.json';
  a.click(); setTimeout(()=>URL.revokeObjectURL(a.href),1000);
}
$('btnMedBefore').onclick = ()=>snapMed('before');
$('btnMedAfter').onclick  = ()=>snapMed('after');
$('btnMedExport').onclick = exportMed;
$('btnMedClear').onclick  = ()=>{ medBefore=null; medAfter=null; renderCompare(); };
renderCompare();

/* ---- Startup ---- */
$('url').value = location.protocol.startsWith('http')
  ? 'ws://' + location.hostname + ':81'          // when served by ESP32, auto-connect to ESP32
  : 'ws://localhost:8765';                    // when opened as a file, connect to your local mock server
setMode('tremor');
setConn('off');
connect();                                    // auto-connect on load; retry every 2 s if it fails
</script>
</body>
</html>

)HTMLDOC";
