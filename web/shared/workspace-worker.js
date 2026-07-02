// 工作空間包絡 Web Worker — geometry.html 即時重算 roll/pitch/yaw 殼，不卡主執行緒 3D。
// 載入共用母函數（SoT 不分叉）；收 {params,clocking,center,grid,thr} → 回 {workspace,safe,stats}。
importScripts('/sysid/workspace_envelope.js');
const WE = self.WorkspaceEnvelope;

self.onmessage = (e) => {
  const { id, params, clocking, center, grid, thr } = e.data;
  try {
    const geo = WE.geoFromParams({ ...params, clocking });
    const env = WE.computeEnvelope(geo, {
      center, grid,
      radialMargin: thr.radialMargin, epsT2: thr.epsT2, kappaThr: thr.kappaThr,
      phys: thr.phys,
      sampleWorkspace: 6000, sampleSafe: 4000,
    });
    self.postMessage({ id, ok: true, workspace: env.workspace, physical: env.physical, safe: env.safe, stats: env.stats });
  } catch (err) {
    self.postMessage({ id, ok: false, error: String(err && err.message || err) });
  }
};
