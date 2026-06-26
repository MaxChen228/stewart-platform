/* ===== Stewart 研究頁共用 client =====
 * 所有 web/research/*.html 共用。封裝實驗頁都會用到的「水電」：
 *   - WebSocket 連線（自動重連）
 *   - 送指令 send()
 *   - 伺服器 JSONL 錄製器 recStart/recStop（單一真相，分析器讀這個）
 *   - 遙測事件 on('telemetry'|'status'|'open'|'close')
 *   - 自動注入統一頂欄（回研究首頁連結 + 標題 + 連線燈）
 *   - 常用統計 helper（Lab.stat.*）
 * 實驗專屬邏輯（推力偵測、狀態機…）留在各自的 *.html，不進這裡。
 */
class Lab {
  constructor({ title = '實驗', host = null } = {}) {
    this.title = title;
    this.host = host || `${location.hostname}:${location.port || 3000}`;
    this.ws = null;
    this.h = { telemetry: [], status: [], open: [], close: [] };
    this._mountHeader();
    this._connect();
  }

  on(evt, fn) { (this.h[evt] || (this.h[evt] = [])).push(fn); return this; }
  _emit(evt, ...a) { (this.h[evt] || []).forEach(fn => { try { fn(...a); } catch (e) { console.error(e); } }); }

  _connect() {
    this.ws = new WebSocket(`ws://${this.host}`);
    this.ws.onopen = () => { this._setWs(true); this._emit('open'); };
    this.ws.onclose = () => { this._setWs(false); this._emit('close'); setTimeout(() => this._connect(), 1500); };
    this.ws.onerror = () => this.ws.close();
    this.ws.onmessage = (ev) => {
      let d; try { d = JSON.parse(ev.data); } catch { return; }
      // 有 a[] 陣列 = 週期遙測；否則是 status/錯誤/診斷回覆
      if (Array.isArray(d.a)) this._emit('telemetry', d);
      else this._emit('status', d);
    };
  }

  send(cmd) { if (this.ws && this.ws.readyState === 1) { this.ws.send(cmd); return true; } return false; }

  // 伺服器錄製器（REST）
  rec(path) { return fetch(`http://${this.host}/api/${path}`).then(r => r.json()).catch(() => ({})); }
  recStart(name) { return this.rec(`rec/start?name=${encodeURIComponent(name)}`); }
  recStop() { return this.rec('rec/stop'); }
  recStatus() { return this.rec('rec/status'); }

  // ---- 統一頂欄 ----
  // 靜態：無 WS 也可注入（中樞頁用）。back={text,href} 自訂返回連結。
  static mountHeader(title, { back = { text: '‹ 研究首頁', href: './' }, showWs = true } = {}) {
    let host = document.getElementById('lab-header');
    if (!host) { host = document.createElement('div'); document.body.prepend(host); }
    host.innerHTML =
      `<div class="lab-top">
         <div class="left">
           <a class="lab-back" href="${back.href}">${back.text}</a>
           <span class="lab-title">${title}</span>
         </div>
         <span class="lab-ws" id="lab-ws" style="${showWs ? '' : 'display:none'}">DISCONNECTED</span>
       </div>`;
    return document.getElementById('lab-ws');
  }
  _mountHeader() { this._wsEl = Lab.mountHeader(this.title); }
  _setWs(ok) {
    if (!this._wsEl) return;
    this._wsEl.textContent = ok ? 'CONNECTED' : 'DISCONNECTED';
    this._wsEl.classList.toggle('ok', ok);
  }

  static sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
}

// 常用統計（純函式，實驗頁與分析共用同一套定義）
Lab.stat = {
  norm: (v) => Math.sqrt(v.reduce((s, x) => s + x * x, 0)),
  median(a) { const s = [...a].sort((x, y) => x - y), m = s.length >> 1;
    return s.length ? (s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2) : 0; },
  mad(a) { const m = Lab.stat.median(a); return Lab.stat.median(a.map(x => Math.abs(x - m))); },
  medianRows(rows) { return rows[0].map((_, j) => Lab.stat.median(rows.map(r => r[j]))); },
};

const $ = (id) => document.getElementById(id);
