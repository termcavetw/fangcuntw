/* 方寸 — service worker
   整個 app（含 3MB 字庫）第一次載入就快取起來，之後完全離線可用。 */
/* 舊網址退休：在 fangcuntw.vercel.app 上，這支 service worker 只清快取、
   把畫面帶到搬家頁（把本機的設定帶去 fangcun.termcavetw.com），然後移除自己。 */
if (self.location.hostname === "fangcuntw.vercel.app") {
  self.addEventListener("install", () => self.skipWaiting());
  self.addEventListener("activate", e => e.waitUntil((async () => {
    for (const k of await caches.keys()) await caches.delete(k);
    await self.clients.claim();
    for (const c of await self.clients.matchAll({ type: "window", includeUncontrolled: true })) {
      const u = new URL(c.url);
      if (u.pathname !== "/migrate.html") { try { await c.navigate("/migrate.html?to=" + encodeURIComponent(u.pathname + u.search + u.hash)); } catch (err) {} }
    }
    await self.registration.unregister();
  })()));
} else {
const CACHE = "fangcun-v268-kagecut";
/* 正楷體字型檔獨立一個快取，換版不清掉（檔案內容永遠不變，免得每次更新都重下 19 MB） */
const KAI_CACHE = "fangcun-twkai-98_1";   /* 換版本號 → 舊快取會被清掉，使用者不用重整兩次 */
/* v265：只快取 "./" 一份。原本 "./" 和 "./index.html" 各存一份完整的 app，等於下載兩次。 */
const CORE_ASSETS = ["./", "./manifest.webmanifest"];
const OPTIONAL_ASSETS = [
  "./icon.svg", "./icon-192.png", "./icon-512.png", "./favicon-32.png", "./apple-touch-icon.png",
  "./icon-mask-192.png", "./icon-mask-512.png"
];

self.addEventListener("install", e => {
  e.waitUntil(
    caches.open(CACHE)
      .then(async c => {
        await c.addAll(CORE_ASSETS);
        await Promise.allSettled(OPTIONAL_ASSETS.map(asset => c.add(asset)));
      })
      .then(() => self.skipWaiting())
      .catch(() => self.skipWaiting())
  );
});

self.addEventListener("activate", e => {
  e.waitUntil(
    caches.keys()
      .then(ks => Promise.all(ks.filter(k => k !== CACHE && k !== KAI_CACHE).map(k => caches.delete(k))))
      .then(() => self.clients.claim())
  );
});

self.addEventListener("fetch", e => {
  const req = e.request;
  if (req.method !== "GET" || new URL(req.url).origin !== location.origin) return;
  const url = new URL(req.url);
  const isPage = req.mode === "navigate" || url.pathname.endsWith("/") ||
                 url.pathname.endsWith("/index.html");

  if (isPage) {
    /* app 本體走「網路優先」：一上線就拿到新版，離線才退回快取 */
    e.respondWith(
      fetch(req).then(res => {
        if (res && res.ok) { const c = res.clone(); caches.open(CACHE).then(k => k.put(req, c)); }
        return res;
      }).catch(() => caches.match(req).then(hit => hit || caches.match("./")))
    );
    return;
  }
  /* v265：正楷體字型檔（19 MB、內容永遠不變）只下載一次：有快取就直接用，不在背景重抓 */
  if (url.pathname.endsWith("/tw-kai-98_1.bin")) {
    e.respondWith(
      caches.match(req).then(hit => hit || fetch(req).then(res => {
        if (res && res.ok) { const c = res.clone(); caches.open(KAI_CACHE).then(k => k.put(req, c)); }
        return res;
      }))
    );
    return;
  }
  /* 圖示等靜態檔走「快取優先」，開啟比較快 */
  e.respondWith(
    caches.match(req).then(hit => {
      const net = fetch(req).then(res => {
        if (res && res.ok) { const c = res.clone(); caches.open(CACHE).then(k => k.put(req, c)); }
        return res;
      }).catch(() => hit);
      return hit || net;
    })
  );
});
}
