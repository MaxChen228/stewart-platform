/* ===== Stewart 監控引擎（單一真相源）=====
 * 六軸示波器的 WS + 環形緩衝 + 繪圖 + 相圖 + 暫停/scrub/拖曳/CSV，全在這裡。
 * scope.html（主控監控）與 research 監控頁共用同一引擎，疊加各自面板。
 *
 * 用法：
 *   const scope = new ScopeCore({
 *     charts: { angles:'c-angles', phase:'c-phase', errors:'c-errors', gain:'c-gain' }, // 要哪些圖
 *     ui: { legend:'legend', stats:'stats', viewPos:'view-pos', scrubber:'scrubber', ... }, // 可選 UI
 *   });
 *   scope.on('telemetry', d => {...});   // 每筆遙測（研究頁加 FFT/σ/CAN）
 *   scope.on('cmd', echo => {...});      // 指令 echo（core 已自動 markEvent）
 *   scope.on('render', () => {...});     // 每幀（研究頁畫自己的面板）
 *   scope.send(cmd);  scope.markEvent(label, kind);  scope.buf;
 *
 * UI 綁定全部容錯：傳入的 id 不存在就跳過 → 同一引擎適配「全 UI 的 scope」與「精簡的研究頁」。
 */
class ScopeCore {
  constructor(cfg = {}) {
    this.cfg = cfg;
    this.host = cfg.host || `${location.hostname}:${location.port || 3000}`;
    this.COLORS = cfg.colors || ['#e74c3c', '#e67e22', '#f1c40f', '#2ecc71', '#3498db', '#9b59b6'];
    this.NAMES = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6'];
    this.POSE_NAMES = ['X', 'Y', 'Z', 'Rol', 'Pit', 'Yaw'];
    this.MAX_BUF = cfg.maxBuf || 3000;
    this.DPR = window.devicePixelRatio || 1;
    this.dt = cfg.dt || 0.02;
    this.charts = cfg.charts || {};
    this.ui = cfg.ui || {};
    this.displayMode = cfg.displayMode || 'motor';

    this.handlers = { telemetry: [], cmd: [], status: [], render: [], open: [], close: [] };
    this.ws = null;
    this.paused = false;
    this.timeWindow = cfg.timeWindow || 250;
    this.selectedMotor = 0;
    this.motorVisible = [true, true, true, true, true, true];
    this.sampleCount = 0;
    this.viewOffset = 0;
    this.events = [];                 // {sample, label, kind}
    this._yrange = {};                // 每圖的穩定 Y 範圍（遲滯，避免每幀抖動）

    const mk6 = () => Array.from({ length: 6 }, () => []);
    this.buf = { a: mk6(), tgt: mk6(), adj: mk6(), vel: mk6(), herr: mk6(), pose: mk6(), poseTgt: mk6(), poseVel: mk6(), g: [], kinetic: [], ts: [], raw: [] };
    this.last = null;                 // 最近一筆原始遙測（研究頁取 ok/tx/herr…）

    if (cfg.autostart !== false) this.start();
  }

  on(evt, fn) { (this.handlers[evt] || (this.handlers[evt] = [])).push(fn); return this; }
  _emit(evt, ...a) { (this.handlers[evt] || []).forEach(f => { try { f(...a); } catch (e) { console.error(e); } }); }
  _el(id) { return id ? document.getElementById(id) : null; }

  // opts.connect===false → 不開自有 WS（由宿主餵 ingest()，單一傳輸源；融合主控頁用）
  start(opts = {}) { this._initUI(); if (opts.connect !== false) this._connectWS(); requestAnimationFrame(() => this._render()); }

  // 單一分派：自有 WS 與外部宿主都走這裡（DRY）。telemetry / cmd echo / status 三類。
  ingest(d) {
    if (Array.isArray(d.a)) { if (!this.paused) this._onData(d); this.last = d; this._emit('telemetry', d); }
    else if (d.evt === 'cmd') { this.markEvent('→ ' + d.c, 'cmd'); this._emit('cmd', d); }   // 對等 echo
    else this._emit('status', d);
  }

  // ===== WebSocket =====
  _connectWS() {
    this.ws = new WebSocket(`ws://${this.host}`);
    this.ws.onopen = () => { const s = this._el(this.ui.status); if (s) s.classList.add('connected'); this._emit('open'); };
    this.ws.onclose = () => { const s = this._el(this.ui.status); if (s) s.classList.remove('connected'); this._emit('close'); setTimeout(() => this._connectWS(), 2000); };
    this.ws.onerror = () => this.ws.close();
    this.ws.onmessage = (ev) => { let d; try { d = JSON.parse(ev.data); } catch { return; } this.ingest(d); };
  }

  _onData(d) {
    if (!d.a || d.a.length !== 6) return;
    this.sampleCount++;
    const b = this.buf;
    const pose = this._poseFromAngles(d.a);
    const poseTgt = this._poseFromAngles(d.tgt);
    for (let i = 0; i < 6; i++) {
      const aArr = b.a[i];
      const prev = aArr.length > 0 ? aArr[aArr.length - 1] : d.a[i];
      aArr.push(d.a[i]);
      b.vel[i].push((d.a[i] - prev) / this.dt);
      b.tgt[i].push(d.tgt ? d.tgt[i] : null);
      b.adj[i].push(d.adj ? d.adj[i] : null);
      b.herr[i].push(Array.isArray(d.herr) ? d.herr[i] : null);
      const pArr = b.pose[i], pPrev = pArr.length > 0 ? pArr[pArr.length - 1] : (pose ? pose[i] : null);
      pArr.push(pose ? pose[i] : null);
      b.poseTgt[i].push(poseTgt ? poseTgt[i] : null);
      b.poseVel[i].push(pose && pPrev != null ? (pose[i] - pPrev) / this.dt : null);
      if (aArr.length > this.MAX_BUF) {
        aArr.shift(); b.vel[i].shift(); b.tgt[i].shift(); b.adj[i].shift(); b.herr[i].shift();
        b.pose[i].shift(); b.poseTgt[i].shift(); b.poseVel[i].shift();
      }
    }
    b.g.push(d.g != null ? d.g : null);
    let kin = 0;
    for (let i = 0; i < 6; i++) { const a = b.a[i]; if (a.length >= 2) { const dd = a[a.length - 1] - a[a.length - 2]; kin += dd * dd; } }
    b.kinetic.push(kin);
    b.ts.push(performance.now());
    b.raw.push(d);
    if (b.g.length > this.MAX_BUF) { b.g.shift(); b.kinetic.shift(); b.ts.shift(); b.raw.shift(); }
  }

  send(cmd) { if (this.ws && this.ws.readyState === 1) { this.ws.send(cmd); return true; } return false; }
  markEvent(label, kind = 'cmd') { this.events.push({ sample: this.buf.a[0].length, label, kind }); if (this.events.length > 300) this.events.shift(); }
  setDisplayMode(mode) {
    this.displayMode = mode === 'pose' ? 'pose' : 'motor';
    this._yrange = {};
    this._updateLegend();
    document.querySelectorAll('[data-scope-mode]').forEach(btn => btn.classList.toggle('active', btn.dataset.scopeMode === this.displayMode));
  }
  _poseFromAngles(angles) {
    if (!Array.isArray(angles) || angles.length !== 6 || typeof this.cfg.poseFromAngles !== 'function') return null;
    try {
      const p = this.cfg.poseFromAngles(angles.slice());
      if (!Array.isArray(p) || p.length !== 6 || !p.every(Number.isFinite)) return null;
      const z0 = Number.isFinite(this.cfg.poseNeutralZ) ? this.cfg.poseNeutralZ : 0;
      return [p[0], p[1], p[2] - z0, p[3], p[4], p[5]];
    } catch (e) { return null; }
  }
  _series() {
    const pose = this.displayMode === 'pose';
    return {
      names: pose ? this.POSE_NAMES : this.NAMES,
      units: pose ? ['mm', 'mm', 'mm', '°', '°', '°'] : ['°', '°', '°', '°', '°', '°'],
      value: pose ? this.buf.pose : this.buf.a,
      target: pose ? this.buf.poseTgt : this.buf.tgt,
      vel: pose ? this.buf.poseVel : this.buf.vel,
      title: pose ? 'Board pose / target (mm, °)' : 'Angles / hold (°)',
      errTitle: pose ? 'Pose error · target−act (mm, °)' : 'Hold error · hold−act (°)',
      phaseX: pose ? 'err' : 'err(°)',
      phaseY: pose ? 'vel' : 'vel(°/s)',
    };
  }

  // ===== 視窗 =====
  // _vStart 不 clamp 到 0：視窗恆 = timeWindow 寬（ve-vs 永遠 = timeWindow）→ 每樣本佔固定像素，
  // buffer 未滿時資料只填右側、從右流入（固定比例流動），不再「先鋪滿全寬再壓縮」。
  // 負 index 由各繪圖迴圈的 null/範圍守衛跳過。
  _vEnd() { return Math.max(0, this.buf.a[0].length - this.viewOffset); }
  _vStart() { return this._vEnd() - this.timeWindow; }

  // ===== Canvas 原語 =====
  _setup(canvas) {
    // 讀 canvas 自身 CSS 尺寸（不回寫 style → 不造成 layout 正反饋）；只在尺寸變時重設 buffer。
    const rect = canvas.getBoundingClientRect();
    const w = Math.max(1, Math.floor(rect.width)), h = Math.max(1, Math.floor(rect.height));
    if (canvas.width !== w * this.DPR || canvas.height !== h * this.DPR) { canvas.width = w * this.DPR; canvas.height = h * this.DPR; }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(this.DPR, 0, 0, this.DPR, 0, 0);   // 絕對變換，避免 scale 每幀累積
    return { ctx, w, h };
  }
  _autoRange(arrays, vs, ve) {
    let min = Infinity, max = -Infinity;
    for (const arr of arrays) {
      const s = Math.max(0, Math.min(vs, arr.length)), e = Math.min(ve, arr.length);
      for (let i = s; i < e; i++) if (arr[i] != null) { if (arr[i] < min) min = arr[i]; if (arr[i] > max) max = arr[i]; }
    }
    if (min === Infinity) { min = 0; max = 1; }
    const pad = Math.max((max - min) * 0.1, 0.5);
    return { min: min - pad, max: max + pad };
  }
  // Y 範圍 snap 到整齊級距（1/2/5×10ⁿ 細分），避免畸零邊界
  _nice(r) {
    let span = r.max - r.min; if (!(span > 0)) span = 1;
    const mag = Math.pow(10, Math.floor(Math.log10(span)));
    const step = (span / mag > 5) ? mag : (span / mag > 2) ? mag / 2 : mag / 5;
    const min = Math.floor(r.min / step) * step, max = Math.ceil(r.max / step) * step;
    return { min, max: max > min ? max : min + step };
  }
  // 穩定範圍：擴張即時、收縮遲滯（跌破當前一半才換檔）→ 範圍不連續抖動
  _stableRange(key, arrays, vs, ve) {
    const raw = this._autoRange(arrays, vs, ve), cur = this._yrange[key];
    if (!cur) return this._yrange[key] = this._nice(raw);
    let { min, max } = cur, changed = false;
    if (raw.min < min) { min = raw.min; changed = true; }
    if (raw.max > max) { max = raw.max; changed = true; }
    if ((raw.max - raw.min) < (max - min) * 0.5) { min = raw.min; max = raw.max; changed = true; }
    if (changed) this._yrange[key] = this._nice({ min, max });
    return this._yrange[key];
  }
  _grid(ctx, w, h, yMin, yMax, tw) {
    ctx.strokeStyle = '#1a1a1a'; ctx.lineWidth = 1;
    const yR = yMax - yMin;
    for (let i = 0; i <= 4; i++) {
      const y = h - (i / 4) * h;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
      ctx.fillStyle = '#444'; ctx.font = '10px monospace'; ctx.textAlign = 'right';
      ctx.fillText((yMin + (i / 4) * yR).toFixed(1), w - 4, y - 2);
    }
    if (yMin < 0 && yMax > 0) {
      const y0 = h - (-yMin / yR) * h;
      ctx.strokeStyle = '#333'; ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.fillStyle = '#333'; ctx.textAlign = 'center';
    const sps = this.dt, totalSec = tw * sps, offSec = this.viewOffset * sps;
    const step = [1, 2, 5, 10, 15, 30].find(t => totalSec / t <= 6) || 10;
    for (let s = step; s < totalSec + step; s += step) {
      const x = w - ((s - offSec % step) / sps / tw) * w, label = s + offSec - (offSec % step);
      if (x < 30 || x > w - 10) continue;
      ctx.fillText(`-${label.toFixed(0)}s`, x, h - 2);
    }
  }
  _trace(ctx, arr, w, h, yMin, yMax, color, vs, ve, lw) {
    const count = ve - vs; if (count < 2) return;
    const yR = yMax - yMin; ctx.strokeStyle = color; ctx.lineWidth = lw || 1.5; ctx.beginPath();
    let first = true;
    for (let i = vs; i < ve && i < arr.length; i++) {
      if (arr[i] == null) { first = true; continue; }
      const x = ((i - vs) / (count - 1)) * w, y = h - ((arr[i] - yMin) / yR) * h;
      if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  // 事件標記線（畫在以 sample 為 x 軸的圖上）
  _marks(ctx, w, h, vs, ve) {
    const count = ve - vs; if (count < 2) return;
    for (const e of this.events) {
      if (e.sample < vs || e.sample > ve) continue;
      const x = ((e.sample - vs) / (count - 1)) * w;
      ctx.strokeStyle = e.kind === 'bad' ? '#a33' : e.kind === 'cmd' ? '#4a8' : '#555';
      ctx.lineWidth = 1; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); ctx.setLineDash([]);
      if (e.kind === 'cmd') { ctx.fillStyle = '#4a8'; ctx.font = '9px monospace'; ctx.textAlign = 'left'; ctx.fillText(e.label.slice(0, 12), x + 2, 20); }
    }
  }

  // ===== 標準圖 =====
  _drawAngles() {
    const cv = this._el(this.charts.angles); if (!cv) return;
    const { ctx, w, h } = this._setup(cv); ctx.clearRect(0, 0, w, h);
    const vs = this._vStart(), ve = this._vEnd();
    const s = this._series();
    const tl = this._el(this.charts.anglesTitle); if (tl) tl.textContent = s.title;
    const visible = []; for (let i = 0; i < 6; i++) if (this.motorVisible[i]) visible.push(s.value[i]);
    const r = this._stableRange('angles', visible, vs, ve);
    this._grid(ctx, w, h, r.min, r.max, ve - vs);
    this._marks(ctx, w, h, vs, ve);
    for (let i = 0; i < 6; i++) {
      if (!this.motorVisible[i]) continue;
      if (s.target[i].some(v => v != null)) {
        ctx.setLineDash([3, 3]); ctx.globalAlpha = 0.3;
        this._trace(ctx, s.target[i], w, h, r.min, r.max, this.COLORS[i], vs, ve, 1);
        ctx.setLineDash([]); ctx.globalAlpha = 1;
      }
      this._trace(ctx, s.value[i], w, h, r.min, r.max, this.COLORS[i], vs, ve, 1.5);
    }
    const idx = Math.min(ve - 1, s.value[0].length - 1), vals = [];
    for (let i = 0; i < 6; i++) if (this.motorVisible[i] && idx >= 0 && idx < s.value[i].length && s.value[i][idx] != null) vals.push(`${s.names[i]}:${s.value[i][idx].toFixed(1)}${s.units[i]}`);
    const l = this._el(this.charts.anglesLabel); if (l) l.textContent = vals.join('  ');
  }
  _drawErrors() {
    const cv = this._el(this.charts.errors); if (!cv) return;
    const { ctx, w, h } = this._setup(cv); ctx.clearRect(0, 0, w, h);
    const s = this._series();
    const tl = this._el(this.charts.errorsTitle); if (tl) tl.textContent = s.errTitle;
    const vs = this._vStart(), ve = this._vEnd(), errors = [];
    for (let i = 0; i < 6; i++) {
      if (!this.motorVisible[i]) continue;
      const err = [];
      for (let j = vs; j < ve && j < s.value[i].length; j++) err.push(s.target[i][j] != null && s.value[i][j] != null ? s.target[i][j] - s.value[i][j] : null);
      errors.push({ idx: i, data: err });
    }
    const r = this._stableRange('errors', errors.map(e => e.data), 0, errors[0] ? errors[0].data.length : 0);
    this._grid(ctx, w, h, r.min, r.max, errors[0] ? errors[0].data.length : this.timeWindow);
    for (const e of errors) this._trace(ctx, e.data, w, h, r.min, r.max, this.COLORS[e.idx], 0, e.data.length, 1.5);
    let rms = 0, cnt = 0;
    for (const e of errors) { const last = e.data[e.data.length - 1]; if (last != null) { rms += last * last; cnt++; } }
    const l = this._el(this.charts.errorsLabel); if (l) l.textContent = `RMS: ${(cnt ? Math.sqrt(rms / cnt) : 0).toFixed(2)}${this.displayMode === 'pose' ? '' : '°'}`;
  }
  _drawGain() {
    const cv = this._el(this.charts.gain); if (!cv) return;
    const { ctx, w, h } = this._setup(cv); ctx.clearRect(0, 0, w, h);
    const vs = this._vStart(), ve = this._vEnd();
    const gR = { min: 0, max: 1.1 }, kR = this._stableRange('kinetic', [this.buf.kinetic], vs, ve); kR.min = 0;
    this._grid(ctx, w, h, gR.min, gR.max, ve - vs);
    const count = ve - vs, i0 = Math.max(0, vs);   // i0：跳過未滿時的負 index（固定比例、從右流入）
    if (count > 1) {
      const kRng = kR.max - kR.min || 1;
      ctx.fillStyle = 'rgba(231,76,60,0.15)'; ctx.beginPath(); let started = false;
      for (let i = i0; i < ve && i < this.buf.kinetic.length; i++) {
        const x = ((i - vs) / (count - 1)) * w, y = h - ((this.buf.kinetic[i] - kR.min) / kRng) * h;
        if (!started) { ctx.moveTo(x, h); ctx.lineTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      if (started) { ctx.lineTo(w, h); ctx.fill(); }
      ctx.strokeStyle = '#c0392b'; ctx.lineWidth = 1; ctx.beginPath();
      let first = true;
      for (let i = i0; i < ve && i < this.buf.kinetic.length; i++) { const x = ((i - vs) / (count - 1)) * w, y = h - ((this.buf.kinetic[i] - kR.min) / kRng) * h; if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y); }
      ctx.stroke();
    }
    this._trace(ctx, this.buf.g, w, h, gR.min, gR.max, '#2ecc71', vs, ve, 2);
    const idx = Math.min(ve - 1, this.buf.g.length - 1);
    const l = this._el(this.charts.gainLabel);
    if (l) l.textContent = `g=${idx >= 0 && this.buf.g[idx] != null ? this.buf.g[idx].toFixed(3) : '—'}  kinetic=${(idx >= 0 ? this.buf.kinetic[idx] || 0 : 0).toFixed(2)}`;
  }
  _drawPhase() {
    const cv = this._el(this.charts.phase); if (!cv) return;
    const { ctx, w, h } = this._setup(cv); ctx.clearRect(0, 0, w, h);
    const s = this._series();
    const mi = this.selectedMotor, aArr = s.value[mi], vArr = s.vel[mi], tArr = s.target[mi];
    const vs = this._vStart(), ve = this._vEnd(); if (ve - vs < 2) return;
    const xs = [], ys = [], i0 = Math.max(0, vs);   // i0：跳過未滿時的負 index
    const base = aArr[i0] ?? 0;
    for (let i = i0; i < ve && i < aArr.length; i++) {
      if (aArr[i] == null) continue;
      xs.push(tArr[i] != null ? tArr[i] - aArr[i] : aArr[i] - base);
      ys.push(vArr[i] || 0);
    }
    let xMax = 0.5, yMax = 10;
    for (let i = 0; i < xs.length; i++) { xMax = Math.max(xMax, Math.abs(xs[i])); yMax = Math.max(yMax, Math.abs(ys[i])); }
    xMax *= 1.2; yMax *= 1.2;
    const cx = w / 2, cy = h / 2;
    ctx.strokeStyle = '#333'; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#555'; ctx.font = '10px monospace'; ctx.textAlign = 'center';
    ctx.fillText(s.phaseX, cx, h - 2);
    ctx.save(); ctx.translate(12, cy); ctx.rotate(-Math.PI / 2); ctx.fillText(s.phaseY, 0, 0); ctx.restore();
    const count = xs.length;
    for (let i = 1; i < count; i++) {
      ctx.strokeStyle = this.COLORS[mi]; ctx.globalAlpha = 0.1 + 0.9 * (i / count); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(cx + (xs[i - 1] / xMax) * (w / 2), cy - (ys[i - 1] / yMax) * (h / 2)); ctx.lineTo(cx + (xs[i] / xMax) * (w / 2), cy - (ys[i] / yMax) * (h / 2)); ctx.stroke();
    }
    ctx.globalAlpha = 1;
    if (count > 0) { ctx.fillStyle = this.COLORS[mi]; ctx.beginPath(); ctx.arc(cx + (xs[count - 1] / xMax) * (w / 2), cy - (ys[count - 1] / yMax) * (h / 2), 4, 0, Math.PI * 2); ctx.fill(); }
    const l = this._el(this.charts.phaseLabel); if (l) l.textContent = `${s.names[mi]}  spiral out=unstable`;
  }

  _render() {
    if (!this.paused) this.viewOffset = 0;
    this._drawAngles(); this._drawErrors(); this._drawGain(); this._drawPhase();
    this._updateScrubber();
    const stats = this._el(this.ui.stats); if (stats) stats.textContent = `${this.sampleCount} samples | ${this.buf.a[0].length} buffered`;
    const pos = this._el(this.ui.viewPos); if (pos) pos.textContent = (this.paused && this.viewOffset > 0) ? `@ -${(this.viewOffset * this.dt).toFixed(1)}s` : '';
    this._emit('render');
    requestAnimationFrame(() => this._render());
  }

  // ===== UI（全部容錯）=====
  _updateScrubber() {
    const scrubber = this._el(this.ui.scrubber), thumb = this._el(this.ui.scrubThumb); if (!scrubber || !thumb) return;
    if (!this.paused) { scrubber.style.display = 'none'; return; }
    scrubber.style.display = 'block';
    const total = this.buf.a[0].length;
    if (total <= this.timeWindow) { thumb.style.left = '0'; thumb.style.width = '100%'; return; }
    const track = this._el(this.ui.scrubTrack); if (!track) return;
    const trackW = track.getBoundingClientRect().width, ratio = this.timeWindow / total, thumbW = Math.max(6, ratio * trackW);
    thumb.style.width = thumbW + 'px';
    thumb.style.left = Math.max(0, ((total - this.viewOffset - this.timeWindow) / (total - this.timeWindow)) * (trackW - thumbW)) + 'px';
  }
  clear() {
    for (let i = 0; i < 6; i++) {
      this.buf.a[i].length = 0; this.buf.tgt[i].length = 0; this.buf.adj[i].length = 0; this.buf.vel[i].length = 0; this.buf.herr[i].length = 0;
      this.buf.pose[i].length = 0; this.buf.poseTgt[i].length = 0; this.buf.poseVel[i].length = 0;
    }
    this.buf.g.length = 0; this.buf.kinetic.length = 0; this.buf.ts.length = 0; this.buf.raw.length = 0; this.events.length = 0; this.sampleCount = 0; this.viewOffset = 0; this._yrange = {};
  }
  togglePause(v) {
    this.paused = (v === undefined) ? !this.paused : v;
    document.body.classList.toggle('paused', this.paused);
    if (!this.paused) this.viewOffset = 0;
    return this.paused;
  }
  exportCSV() {
    const n = this.buf.a[0].length; if (n === 0) return;
    let csv = 'sample,a1,a2,a3,a4,a5,a6,tgt1,tgt2,tgt3,tgt4,tgt5,tgt6,herr1,herr2,herr3,herr4,herr5,herr6,v1,v2,v3,v4,v5,v6,pose_x,pose_y,pose_z_off,pose_roll,pose_pitch,pose_yaw,pose_tgt_x,pose_tgt_y,pose_tgt_z_off,pose_tgt_roll,pose_tgt_pitch,pose_tgt_yaw,gain,kinetic\n';
    for (let j = 0; j < n; j++) {
      const row = [j];
      for (let i = 0; i < 6; i++) row.push(this.buf.a[i][j]?.toFixed(3) ?? '');
      for (let i = 0; i < 6; i++) row.push(this.buf.tgt[i][j]?.toFixed(3) ?? '');
      for (let i = 0; i < 6; i++) row.push(this.buf.herr[i][j]?.toFixed(3) ?? '');
      for (let i = 0; i < 6; i++) row.push(this.buf.vel[i][j]?.toFixed(3) ?? '');
      for (let i = 0; i < 6; i++) row.push(this.buf.pose[i][j]?.toFixed(3) ?? '');
      for (let i = 0; i < 6; i++) row.push(this.buf.poseTgt[i][j]?.toFixed(3) ?? '');
      row.push(this.buf.g[j]?.toFixed(4) ?? ''); row.push(this.buf.kinetic[j]?.toFixed(4) ?? '');
      csv += row.join(',') + '\n';
    }
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
    a.download = `scope_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.csv`;
    a.click();
  }
  currentWindowBundle() {
    const n = this.buf.a[0].length;
    if (!n) return null;
    const vs = this._vStart(), ve = this._vEnd();
    const start = Math.max(0, Math.min(n, vs));
    const end = Math.max(start, Math.min(n, ve));
    const t0 = this.buf.ts[start] ?? this.buf.ts[0] ?? 0;
    const samples = [];
    const finite = (v) => Number.isFinite(Number(v)) ? Number(v) : null;
    for (let j = start; j < end; j++) {
      samples.push({
        sample: j,
        tRelMs: finite((this.buf.ts[j] ?? t0) - t0),
        raw: this.buf.raw[j] || null,
        motors: {
          angle: this.buf.a.map((x) => finite(x[j])),
          target: this.buf.tgt.map((x) => finite(x[j])),
          holdError: this.buf.herr.map((x) => finite(x[j])),
          velocity: this.buf.vel.map((x) => finite(x[j])),
        },
        motor: {
          angle: this.buf.a.map((x) => finite(x[j])),
          target: this.buf.tgt.map((x) => finite(x[j])),
          holdError: this.buf.herr.map((x) => finite(x[j])),
          velocity: this.buf.vel.map((x) => finite(x[j])),
        },
        pose6dof: {
          names: this.POSE_NAMES,
          units: ['mm', 'mm', 'mm', 'deg', 'deg', 'deg'],
          actual: this.buf.pose.map((x) => finite(x[j])),
          target: this.buf.poseTgt.map((x) => finite(x[j])),
          velocity: this.buf.poseVel.map((x) => finite(x[j])),
        },
        pose: {
          actual: this.buf.pose.map((x) => finite(x[j])),
          target: this.buf.poseTgt.map((x) => finite(x[j])),
          velocity: this.buf.poseVel.map((x) => finite(x[j])),
        },
        gain: finite(this.buf.g[j]),
        kinetic: finite(this.buf.kinetic[j]),
      });
    }
    const arrStats = (vals) => {
      const xs = vals.map(Number).filter(Number.isFinite);
      if (!xs.length) return { n: 0, min: null, max: null, mean: null, rms: null, pp: null };
      let sum = 0, sum2 = 0, min = Infinity, max = -Infinity;
      for (const x of xs) { sum += x; sum2 += x * x; if (x < min) min = x; if (x > max) max = x; }
      return { n: xs.length, min, max, mean: sum / xs.length, rms: Math.sqrt(sum2 / xs.length), pp: max - min };
    };
    const motorStats = this.NAMES.map((name, i) => ({
      name,
      angle: arrStats(samples.map((s) => s.motor.angle[i])),
      holdError: arrStats(samples.map((s) => s.motor.holdError[i])),
      velocity: arrStats(samples.map((s) => s.motor.velocity[i])),
    }));
    const poseStats = this.POSE_NAMES.map((name, i) => ({
      name,
      unit: i < 3 ? 'mm' : 'deg',
      actual: arrStats(samples.map((s) => s.pose6dof.actual[i])),
      target: arrStats(samples.map((s) => s.pose6dof.target[i])),
      error: arrStats(samples.map((s) => {
        const target = s.pose6dof.target[i], actual = s.pose6dof.actual[i];
        return target != null && actual != null ? target - actual : null;
      })),
      velocity: arrStats(samples.map((s) => s.pose6dof.velocity[i])),
    }));
    return {
      schema: 'stewart.scope.window.v2',
      createdAt: new Date().toISOString(),
      source: location.href,
      displayMode: this.displayMode,
      contains: ['six_motors', 'six_dof_pose', 'raw_telemetry'],
      selectedMotor: this.NAMES[this.selectedMotor],
      visibleMotors: this.NAMES.filter((_, i) => this.motorVisible[i]),
      window: {
        startSample: start,
        endSample: end,
        samples: end - start,
        requestedSamples: this.timeWindow,
        viewOffsetSamples: this.viewOffset,
        estimatedDurationMs: samples.length > 1 ? samples[samples.length - 1].tRelMs : 0,
      },
      summary: {
        motors: motorStats,
        pose6dof: poseStats,
        motor: motorStats,
        pose: poseStats,
        kinetic: arrStats(samples.map((s) => s.kinetic)),
        gain: arrStats(samples.map((s) => s.gain)),
      },
      events: this.events
        .filter((e) => e.sample >= start && e.sample <= end)
        .map((e) => ({ ...e, sampleInWindow: e.sample - start })),
      samples,
    };
  }
  exportWindowJSON() {
    const bundle = this.currentWindowBundle();
    if (!bundle) return null;
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([JSON.stringify(bundle, null, 2)], { type: 'application/json' }));
    a.download = `scope_window_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.json`;
    a.click();
    return bundle;
  }

  _initUI() {
    this._updateLegend();
    document.querySelectorAll('[data-scope-mode]').forEach(btn => btn.addEventListener('click', () => this.setDisplayMode(btn.dataset.scopeMode)));
    // 時窗
    document.querySelectorAll('[data-tw]').forEach(btn => btn.addEventListener('click', () => {
      document.querySelectorAll('[data-tw]').forEach(b => b.classList.remove('active')); btn.classList.add('active'); this.timeWindow = parseInt(btn.dataset.tw);
    }));
    // 暫停/clear/csv
    const bp = this._el(this.ui.pauseBtn);
    if (bp) bp.addEventListener('click', () => { const p = this.togglePause(); bp.textContent = p ? '▶ Resume' : '⏸ Pause'; bp.classList.toggle('active', p); });
    const bc = this._el(this.ui.clearBtn); if (bc) bc.addEventListener('click', () => this.clear());
    const bcsv = this._el(this.ui.csvBtn); if (bcsv) bcsv.addEventListener('click', () => this.exportCSV());
    const bdata = this._el(this.ui.dataBtn); if (bdata) bdata.addEventListener('click', () => this.exportWindowJSON());
    // 指令列
    const ci = this._el(this.ui.cmdInput), bs = this._el(this.ui.sendBtn);
    if (ci) {
      const doSend = () => { const c = ci.value.trim(); if (c) { this.send(c); ci.value = ''; } };
      if (bs) bs.addEventListener('click', doSend);
      ci.addEventListener('keydown', (e) => { if (e.key === 'Enter') doSend(); });
    }
    // 鍵盤
    document.addEventListener('keydown', (e) => {
      if (ci && e.target === ci) return;
      if (/^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;   // 別在宿主表單欄位裡攔截空白/方向/數字鍵
      if (e.key === ' ' && bp) { e.preventDefault(); bp.click(); }
      if (e.key >= '1' && e.key <= '6') this.selectedMotor = parseInt(e.key) - 1;
      if (!this.paused) return;
      const step = e.shiftKey ? Math.round(this.timeWindow / 4) : 5, maxOff = Math.max(0, this.buf.a[0].length - this.timeWindow);
      if (e.key === 'ArrowLeft') { this.viewOffset = Math.min(maxOff, this.viewOffset + step); e.preventDefault(); }
      if (e.key === 'ArrowRight') { this.viewOffset = Math.max(0, this.viewOffset - step); e.preventDefault(); }
      if (e.key === 'Home') { this.viewOffset = maxOff; e.preventDefault(); }
      if (e.key === 'End') { this.viewOffset = 0; e.preventDefault(); }
    });
    this._initDragPan(); this._initScrubberDrag();
  }
  _updateLegend() {
    const legend = this._el(this.ui.legend); if (!legend) return;
    const names = this._series().names;
    legend.innerHTML = '';
    for (let i = 0; i < 6; i++) {
      const el = document.createElement('div');
      el.className = 'leg-item' + (this.motorVisible[i] ? '' : ' hidden');
      el.innerHTML = `<div class="leg-dot" style="background:${this.COLORS[i]}"></div><span>${names[i]}</span>`;
      el.addEventListener('click', () => { this.motorVisible[i] = !this.motorVisible[i]; el.classList.toggle('hidden', !this.motorVisible[i]); });
      el.addEventListener('dblclick', (e) => { e.preventDefault(); this.selectedMotor = i; });
      legend.appendChild(el);
    }
  }
  _initDragPan() {
    const chartsEl = this._el(this.ui.charts); if (!chartsEl) return;
    let drag = null;
    chartsEl.addEventListener('mousedown', (e) => { if (!this.paused || e.target.closest('.phase')) return; drag = { startX: e.clientX, startOffset: this.viewOffset }; document.body.classList.add('dragging'); e.preventDefault(); });
    window.addEventListener('mousemove', (e) => { if (!drag) return; const cw = this._el(this.charts.angles).parentElement.getBoundingClientRect().width; const dx = e.clientX - drag.startX, maxOff = Math.max(0, this.buf.a[0].length - this.timeWindow); this.viewOffset = Math.max(0, Math.min(maxOff, drag.startOffset + Math.round(dx * (this.timeWindow / cw)))); });
    window.addEventListener('mouseup', () => { if (drag) { drag = null; document.body.classList.remove('dragging'); } });
    chartsEl.addEventListener('wheel', (e) => { if (!this.paused || e.target.closest('.phase')) return; e.preventDefault(); const delta = Math.sign(e.deltaX || e.deltaY) * Math.max(1, Math.round(this.timeWindow * 0.05)), maxOff = Math.max(0, this.buf.a[0].length - this.timeWindow); this.viewOffset = Math.max(0, Math.min(maxOff, this.viewOffset + delta)); }, { passive: false });
  }
  _initScrubberDrag() {
    const track = this._el(this.ui.scrubTrack); if (!track) return;
    let dragging = false;
    const scrubTo = (clientX) => { const rect = track.getBoundingClientRect(), ratio = Math.max(0, Math.min(1, (clientX - rect.left) / rect.width)), maxOff = Math.max(0, this.buf.a[0].length - this.timeWindow); this.viewOffset = Math.round(maxOff * (1 - ratio)); };
    track.addEventListener('mousedown', (e) => { dragging = true; scrubTo(e.clientX); e.preventDefault(); });
    window.addEventListener('mousemove', (e) => { if (dragging) scrubTo(e.clientX); });
    window.addEventListener('mouseup', () => { dragging = false; });
  }
}
window.ScopeCore = ScopeCore;
