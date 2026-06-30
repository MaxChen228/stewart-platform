// 頂部導覽列「單一真相源」。
// 連結清單只在這裡定義一次；各頁只放空的 <nav class="app-nav" data-app-nav> 掛載點 +
// <script src="/shared/app-nav.js" defer>。增刪/改名只動本檔，active 用 pathname 算出、不手寫。
(function () {
  const LINKS = [
    { href: '/',                   label: '主控',      title: '定點控制（設目標 → GO，走 P 最優軌跡）' },
    { href: '/live.html',          label: '即時',      title: '即時串流（拖滑桿／手機／模擬）' },
    { href: '/workspace.html',     label: 'Workspace', title: 'Program Workspace' },
    { href: '/research/runs.html', label: 'Runs',      title: 'Session 執行紀錄' },
    { href: '/geometry.html',      label: 'Geometry',  title: '幾何設計／最佳化' },
  ];
  // 正規化：'' 與 '/index.html' 都視為 '/'，其餘照原 pathname 精確比對。
  const norm = (p) => (p === '' || p === '/index.html') ? '/' : p;
  const here = norm(location.pathname);
  document.querySelectorAll('nav.app-nav[data-app-nav]').forEach((nav) => {
    nav.replaceChildren(...LINKS.map((l) => {
      const a = document.createElement('a');
      a.className = 'app-link' + (norm(l.href) === here ? ' active' : '');
      a.href = l.href;
      a.title = l.title;
      a.textContent = l.label;
      return a;
    }));
  });
})();
