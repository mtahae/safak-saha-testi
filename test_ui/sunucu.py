"""
ŞAFAK UAV - Saha Testi Canlı İzleme Arayüzü
============================================
Yazılım uçarken ne yaptığını tarayıcıdan izlemek için. `--ui` ile açılır.

    python -m src.gorev.gorev2 --kaynak picam --sadece-mavi --temsili-servo --ui
    -> tarayicidan  http://<raspi-ip>:5000

TASARIM KURALI: Bu modül görev akışına HİÇ dokunmaz, yalnızca OKUR. Arayüz
çökerse, tarayıcı kapanırsa, ağ giderse görev aynen devam eder. Uçuş kritik
yol ile izleme yolu birbirine bağlı değildir — hocanın kodunda her şeyin tek
süreçte ve tek kilitte olması tam olarak bu yüzden sorun çıkarıyordu.

Neden Flask + MJPEG: RPi'de kurulum derdi yok, tarayıcı eklentisi gerekmez,
telefondan da açılır. Kanıt yayını (yayin.py, UDP) bundan AYRIDIR ve yarışma
günü asıl kullanılacak olan odur; bu arayüz saha testi içindir.
"""
import json
import threading
import time

import cv2

try:
    from flask import Flask, Response, jsonify
except ImportError:  # Flask yoksa görev yine çalışsın
    Flask = None


SAYFA = """<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ŞAFAK UAV — Saha Testi</title>
<style>
:root{
  --zemin:#0e1116; --panel:#161b22; --panel2:#1c232c; --cizgi:#2a323d;
  --yazi:#e6edf3; --yazi2:#9aa7b4; --yazi3:#6e7b8a;
  --iyi:#3fb950; --uyari:#d29922; --kotu:#f85149; --mavi:#58a6ff; --kirmizi:#ff7b72;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--zemin);color:var(--yazi);
  font:14px/1.5 ui-sans-serif,system-ui,"Segoe UI",Roboto,sans-serif}
.mono{font-family:ui-monospace,"Cascadia Mono",Consolas,monospace;
  font-variant-numeric:tabular-nums}
header{background:var(--panel);border-bottom:1px solid var(--cizgi);
  padding:10px 16px;display:flex;align-items:center;gap:14px;flex-wrap:wrap}
h1{font-size:15px;font-weight:700;letter-spacing:.5px}
.bayrak{font-size:11px;font-weight:700;letter-spacing:.08em;padding:3px 9px;
  border-radius:3px;background:#3d2b00;color:var(--uyari);border:1px solid #5c4200}
.faz{margin-left:auto;font-size:13px;font-weight:700;color:var(--mavi);
  letter-spacing:.05em}
main{display:grid;gap:12px;padding:12px;grid-template-columns:1fr;
  max-width:1600px;margin:0 auto}
@media(min-width:1100px){main{grid-template-columns:minmax(0,1.35fr) minmax(360px,1fr)}}
.kart{background:var(--panel);border:1px solid var(--cizgi);border-radius:6px;
  overflow:hidden}
.kart h2{font-size:11px;font-weight:700;letter-spacing:.1em;color:var(--yazi3);
  text-transform:uppercase;padding:9px 12px;border-bottom:1px solid var(--cizgi);
  background:var(--panel2)}
.kart .ic{padding:12px}
#video{width:100%;display:block;background:#000}
.izgara{display:grid;grid-template-columns:repeat(auto-fit,minmax(105px,1fr));gap:1px;
  background:var(--cizgi)}
.hucre{background:var(--panel);padding:9px 11px}
.hucre .et{font-size:10px;letter-spacing:.08em;color:var(--yazi3);
  text-transform:uppercase}
.hucre .dg{font-size:17px;font-weight:600;margin-top:2px}
table{width:100%;border-collapse:collapse;font-size:13px}
th{font-size:10px;letter-spacing:.07em;color:var(--yazi3);text-transform:uppercase;
  text-align:left;padding:6px 10px;border-bottom:1px solid var(--cizgi)}
td{padding:6px 10px;border-bottom:1px solid #1e252e}
tr:last-child td{border-bottom:none}
td.sag,th.sag{text-align:right}
.rozet{font-size:10px;font-weight:700;letter-spacing:.05em;padding:2px 7px;
  border-radius:3px;white-space:nowrap}
.r-onay{background:#0d2f1a;color:var(--iyi)}
.r-bek{background:#3d2b00;color:var(--uyari)}
.r-birak{background:#0d2847;color:var(--mavi)}
.iyi{color:var(--iyi)} .uyari{color:var(--uyari)} .kotu{color:var(--kotu)}
.mavi{color:var(--mavi)} .kirmizi{color:var(--kirmizi)} .sonuk{color:var(--yazi3)}
#birakmalar li{list-style:none;padding:8px 12px;border-bottom:1px solid #1e252e;
  font-size:13px}
#birakmalar li:last-child{border-bottom:none}
.bos{color:var(--yazi3);padding:14px 12px;font-size:13px}
#pankart{display:none;padding:14px 16px;background:#0d2847;color:var(--mavi);
  border:1px solid #1f4a7d;border-radius:6px;font-size:16px;font-weight:700;
  margin:0 12px;text-align:center;letter-spacing:.03em}
.elenen{display:flex;flex-wrap:wrap;gap:6px;font-size:11px}
.elenen span{background:var(--panel2);border:1px solid var(--cizgi);
  border-radius:3px;padding:2px 7px;color:var(--yazi2)}
</style></head><body>

<header>
  <h1>ŞAFAK UAV</h1>
  <div id="bayraklar" style="display:flex;gap:6px;flex-wrap:wrap"></div>
  <div class="faz" id="faz">—</div>
</header>

<div id="pankart"></div>

<main>
  <div style="display:flex;flex-direction:column;gap:12px;min-width:0">
    <div class="kart">
      <h2>Canlı görüntü — işlenmiş kare</h2>
      <img id="video" src="/video" alt="canli goruntu">
    </div>
    <div class="kart">
      <h2>Uçuş durumu</h2>
      <div class="izgara" id="telemetri"></div>
    </div>
  </div>

  <div style="display:flex;flex-direction:column;gap:12px;min-width:0">
    <div class="kart">
      <h2>Hedef havuzu</h2>
      <div id="havuz"></div>
    </div>
    <div class="kart">
      <h2>Bırakmalar</h2>
      <ul id="birakmalar"></ul>
    </div>
    <div class="kart">
      <h2>Algılama sayaçları</h2>
      <div class="ic">
        <div class="izgara" id="algi" style="margin:-12px -12px 10px"></div>
        <div class="elenen" id="elenen"></div>
      </div>
    </div>
  </div>
</main>

<script>
const $ = s => document.querySelector(s);
let sonBirakma = 0;

function hucre(et, dg, sinif){
  return `<div class="hucre"><div class="et">${et}</div>
          <div class="dg mono ${sinif||''}">${dg}</div></div>`;
}

async function guncelle(){
  let d;
  try { d = await (await fetch('/durum')).json(); }
  catch(e){ $('#faz').textContent = 'BAGLANTI YOK'; $('#faz').className='faz kotu'; return; }

  $('#faz').textContent = d.faz;
  $('#faz').className = 'faz';

  $('#bayraklar').innerHTML = d.bayraklar
    .map(b => `<span class="bayrak">${b}</span>`).join('');

  const t = d.telemetri;
  $('#telemetri').innerHTML =
      hucre('Mod', t.mod, t.mod==='GUIDED' ? 'mavi' : '')
    + hucre('Arm', t.armed ? 'ARMED' : 'disarm', t.armed ? 'iyi':'sonuk')
    + hucre('İrtifa', t.irtifa.toFixed(1)+' m')
    + hucre('GPS fix', t.gps_fix + ' / ' + t.uydu + ' uydu',
            t.gps_fix>=3 ? 'iyi' : 'kotu')
    + hucre('Yer hızı', t.yer_hizi.toFixed(2)+' m/s')
    + hucre('Batarya', t.batarya.toFixed(1)+' V')
    + hucre('Waypoint', t.wp)
    + hucre('Konum', t.lat ? t.lat.toFixed(6)+'\\n'+t.lon.toFixed(6) : 'YOK',
            t.lat ? '' : 'kotu');

  $('#algi').innerHTML =
      hucre('FPS', d.algi.fps.toFixed(1))
    + hucre('İşlenen kare', d.algi.kare)
    + hucre('Geçerli tespit', d.algi.gecerli, d.algi.gecerli>0?'iyi':'sonuk')
    + hucre('Tespit', d.algi.acik ? 'AÇIK' : 'kapalı',
            d.algi.acik ? 'iyi' : 'uyari');
  $('#elenen').innerHTML = Object.entries(d.algi.elenen)
    .filter(([k,v]) => v>0)
    .map(([k,v]) => `<span>${k}: ${v}</span>`).join('') || '<span>eleme yok</span>';

  if(d.havuz.length){
    $('#havuz').innerHTML = `<table><thead><tr>
        <th>Hedef</th><th class="sag">Ölçüm</th><th class="sag">Dağılım</th>
        <th>Konum</th><th>Durum</th></tr></thead><tbody>` +
      d.havuz.map(k => `<tr>
        <td class="${k.sinif.startsWith('mavi')?'mavi':'kirmizi'}">${k.sinif}</td>
        <td class="sag mono">${k.n}</td>
        <td class="sag mono">${k.dagilim.toFixed(2)} m</td>
        <td class="mono" style="font-size:11px">${k.lat.toFixed(6)}<br>${k.lon.toFixed(6)}</td>
        <td><span class="rozet ${k.birakildi?'r-birak':(k.onayli?'r-onay':'r-bek')}">${
            k.birakildi?'BIRAKILDI':(k.onayli?'ONAYLI':'bekliyor')}</span></td>
      </tr>`).join('') + '</tbody></table>';
  } else {
    $('#havuz').innerHTML = '<div class="bos">Henüz hedef ölçümü yok.</div>';
  }

  if(d.birakmalar.length){
    $('#birakmalar').innerHTML = d.birakmalar.map(b => `<li>
      <span class="${b.sinif.startsWith('mavi')?'mavi':'kirmizi'}"><b>${b.sinif}</b></span>
      &larr; ${b.yuk}
      <div class="mono sonuk" style="font-size:12px;margin-top:3px">
        hata <span class="${b.hata<0.5?'iyi':(b.hata<1.0?'uyari':'kotu')}">${b.hata.toFixed(2)} m</span>
        &middot; irtifa ${b.irtifa.toFixed(1)} m
        &middot; hız ${b.hiz.toFixed(2)} m/s
        &middot; ${b.zaman.toFixed(0)}. sn</div></li>`).join('');
    if(d.birakmalar.length > sonBirakma){
      sonBirakma = d.birakmalar.length;
      const s = d.birakmalar[d.birakmalar.length-1];
      const p = $('#pankart');
      p.textContent = `YÜK BIRAKILDI — ${s.sinif} · hata ${s.hata.toFixed(2)} m`;
      p.style.display = 'block';
      setTimeout(()=>{ p.style.display='none'; }, 12000);
    }
  } else {
    $('#birakmalar').innerHTML = '<li class="bos">Henüz bırakma yok.</li>';
  }
}
guncelle();
setInterval(guncelle, 500);
</script></body></html>"""


class Arayuz:
    """Görev nesnesini OKUR, hiçbir şeyini değiştirmez."""

    def __init__(self, gorev, port=5000, kalite=70, genislik=960):
        self.g = gorev
        self.port = port
        self.kalite = kalite
        self.genislik = genislik
        self._app = None

    def basla(self):
        if Flask is None:
            self.g.log("[ui] Flask kurulu degil (pip install flask); arayuz yok")
            return
        app = Flask(__name__)
        app.logger.disabled = True
        import logging
        logging.getLogger("werkzeug").setLevel(logging.ERROR)

        @app.route("/")
        def _kok():
            return SAYFA

        @app.route("/durum")
        def _durum():
            return jsonify(self.durum())

        @app.route("/video")
        def _video():
            return Response(self._kareler(),
                            mimetype="multipart/x-mixed-replace; boundary=kare")

        self._app = app
        threading.Thread(target=self._kos, daemon=True).start()
        self.g.log(f"[ui] arayuz hazir -> http://<raspi-ip>:{self.port}")

    def _kos(self):
        try:
            self._app.run(host="0.0.0.0", port=self.port, debug=False,
                          use_reloader=False, threaded=True)
        except Exception as e:
            self.g.log(f"[ui] sunucu durdu (gorev etkilenmez): {e}")

    # ------------------------------------------------------------------
    def _kareler(self):
        """MJPEG akışı. Kare yoksa akış durmaz, bekler."""
        while True:
            kare = None
            try:
                kare = self.g.algi.kare_al()
            except Exception:
                pass
            if kare is None:
                time.sleep(0.1)
                continue
            h, w = kare.shape[:2]
            if w > self.genislik:
                o = self.genislik / w
                kare = cv2.resize(kare, (self.genislik, int(h * o)))
            ok, buf = cv2.imencode(".jpg", kare,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.kalite])
            if ok:
                yield (b"--kare\r\nContent-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
            time.sleep(1 / 12.0)

    def durum(self):
        g = self.g
        d = g.ucus.durum()

        bayraklar = []
        if getattr(g, "sadece_mavi", False):
            bayraklar.append("SADECE MAVİ")
        if getattr(g, "temsili_servo", False):
            bayraklar.append("TEMSİLİ SERVO")
        if g.prova:
            bayraklar.append("PROVA — UÇUŞ KOMUTU YOK")
        if g.ucus.devralindi:
            bayraklar.append("PİLOT DEVRALDI")

        havuz = []
        for k in sorted(g.havuz.kumeler, key=lambda c: -c.sayi):
            lat, lon = k.konum
            havuz.append({
                "sinif": k.sinif, "n": k.sayi,
                "dagilim": round(k.dagilim_m, 3),
                "lat": lat, "lon": lon,
                "onayli": k.sayi >= g.havuz.min_tespit,
                "birakildi": k.birakildi,
            })

        return {
            "faz": getattr(g, "faz", "?"),
            "bayraklar": bayraklar,
            "gecen_s": round(g.gecen, 1),
            "telemetri": {
                "mod": d.mod, "armed": bool(d.armed),
                "irtifa": float(d.irtifa or 0.0),
                "gps_fix": int(d.gps_fix), "uydu": int(d.uydu),
                "batarya": float(d.batarya_v), "yer_hizi": float(d.yer_hizi),
                "wp": int(d.wp_no), "lat": d.lat, "lon": d.lon,
            },
            "algi": {
                "fps": float(g.algi.fps),
                "kare": int(g.algi.kare_sayisi),
                "gecerli": int(g.algi.gecerli_tespit),
                "acik": bool(g.algi.tespit_acik),
                "elenen": dict(g.algi.elenen),
            },
            "havuz": havuz,
            "birakmalar": [
                {"sinif": b["sinif"], "yuk": b["yuk"], "hata": b["hata_m"],
                 "irtifa": b["irtifa"], "hiz": b["hiz"], "zaman": b["zaman_s"]}
                for b in g.birakma_kayit
            ],
        }
