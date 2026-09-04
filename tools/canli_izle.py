"""
ŞAFAK UAV - Canlı Model İzleme (Pixhawk GEREKMEZ)
==================================================
Sadece modeli çalıştırır ve tarayıcıdan canlı gösterir. Uçuş kontrolcüsüne
bağlanmaz, hiçbir komut göndermez, geolokasyon yapmaz.

Ne işe yarar: dronu ELLE uçururken modelin brandayı görüp görmediğini
izlemek. Görev yazılımını denemeden önce "model havadan çalışıyor mu"
sorusunu cevaplar.

    python tools/canli_izle.py --motor hailo        # AI HAT+ (safak_v2.hef)
    python tools/canli_izle.py --motor ncnn         # CPU (picamera2)
    python tools/canli_izle.py --motor hailo --port 5050

Sonra tarayicidan:  http://<raspi-ip>:5000
"""
import argparse
import socket
import sys
import threading
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import cv2

from src.gorev import gorev_config as cfg

DURUM = {"fps": 0.0, "kare": 0, "kirmizi": 0, "mavi": 0,
         "son": "-", "bilinmeyen": [], "motor": "-"}
_kilit = threading.Lock()
_kare = {"bgr": None}


SAYFA = """<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ŞAFAK UAV — Canlı Model</title>
<style>
body{margin:0;background:#0e1116;color:#e6edf3;
  font:14px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{background:#161b22;border-bottom:1px solid #2a323d;padding:10px 16px;
  display:flex;gap:14px;align-items:center;flex-wrap:wrap}
h1{font-size:15px;font-weight:700;letter-spacing:.5px;margin:0}
.et{font-size:11px;font-weight:700;letter-spacing:.08em;padding:3px 9px;
  border-radius:3px;background:#0d2847;color:#58a6ff}
.sar{max-width:1200px;margin:0 auto;padding:12px;display:flex;
  flex-direction:column;gap:12px}
img{width:100%;display:block;background:#000;border:1px solid #2a323d;
  border-radius:6px}
.izgara{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));
  gap:1px;background:#2a323d;border:1px solid #2a323d;border-radius:6px;
  overflow:hidden}
.h{background:#161b22;padding:10px 12px}
.h .k{font-size:10px;letter-spacing:.08em;color:#6e7b8a;text-transform:uppercase}
.h .v{font-size:20px;font-weight:600;margin-top:2px;
  font-family:ui-monospace,Consolas,monospace;font-variant-numeric:tabular-nums}
.mavi{color:#58a6ff}.kirmizi{color:#ff7b72}.iyi{color:#3fb950}.sonuk{color:#6e7b8a}
.uyari{background:#3d2b00;color:#e0ae5e;padding:10px 12px;border-radius:6px;
  font-size:13px;display:none}
</style></head><body>
<header><h1>ŞAFAK UAV</h1><span class="et">CANLI MODEL — PIXHAWK YOK</span>
<span id="motor" class="et" style="background:#0d2f1a;color:#3fb950"></span></header>
<div class="sar">
  <img id="v" src="/video" alt="canli">
  <div class="izgara" id="s"></div>
  <div class="uyari" id="u"></div>
</div>
<script>
const $=s=>document.querySelector(s);
function h(k,v,c){return `<div class="h"><div class="k">${k}</div>
  <div class="v ${c||''}">${v}</div></div>`}
setInterval(async()=>{
  let d; try{ d=await (await fetch('/durum')).json() }catch(e){ return }
  $('#motor').textContent = d.motor;
  $('#s').innerHTML = h('FPS', d.fps.toFixed(1))
    + h('İşlenen kare', d.kare)
    + h('Mavi tespit', d.mavi, d.mavi>0?'mavi':'sonuk')
    + h('Kırmızı tespit', d.kirmizi, d.kirmizi>0?'kirmizi':'sonuk')
    + h('Şu an', d.son, d.son==='-'?'sonuk':'iyi');
  const u=$('#u');
  if(d.bilinmeyen.length){
    u.style.display='block';
    u.innerHTML='<b>BİLİNMEYEN ETİKET:</b> '+d.bilinmeyen.join(', ')
      +' — labels-json yüklenmemiş olabilir, tespitler sayılmıyor.';
  } else u.style.display='none';
}, 500);
</script></body></html>"""


def yerel_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 1))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def dongu(kaynak, ded, ciz_fn):
    """Kare al -> işle -> çiz -> paylaş. Tek iş parçacığı."""
    fps = 0.0
    while True:
        t0 = time.time()
        ok, kare = kaynak.oku()
        if not ok or kare is None:
            time.sleep(0.05)
            continue
        sonuc = ded.isle(kare)
        gorsel = ciz_fn(kare, sonuc)

        sayac = Counter(h.sinif_isim for h in sonuc["tum"])
        etiket = " + ".join(f"{a} {sayac[a]}" for a in sorted(sayac)) or "-"
        cv2.putText(gorsel, time.strftime("%H:%M:%S") + f"  {fps:4.1f} FPS  {etiket}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(gorsel, time.strftime("%H:%M:%S") + f"  {fps:4.1f} FPS  {etiket}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        dt = time.time() - t0
        fps = 0.9 * fps + 0.1 * (1.0 / max(dt, 1e-3))
        with _kilit:
            _kare["bgr"] = gorsel
            DURUM["fps"] = fps
            DURUM["kare"] += 1
            DURUM["kirmizi"] += sayac.get("kirmizi_hedef", 0)
            DURUM["mavi"] += sayac.get("mavi_hedef", 0)
            DURUM["son"] = etiket
            DURUM["bilinmeyen"] = sorted(
                getattr(kaynak, "bilinmeyen_etiket", {}).keys())[:5]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", choices=["hailo", "ncnn", "pt"], default="hailo")
    ap.add_argument("--kaynak", default="picam", help="ncnn/pt icin kamera")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--conf", type=float, default=None)
    args = ap.parse_args()
    conf = args.conf if args.conf is not None else cfg.MIN_GUVEN

    if args.motor == "hailo":
        from src.detect_hailo import HailoKaynak
        print(f"[izle] Hailo baslatiliyor: {cfg.HAILO_HEF}")
        k = HailoKaynak(cfg.HAILO_HEF, cfg.HAILO_ETIKET_JSON,
                        kaynak=cfg.HAILO_KAYNAK, conf=conf,
                        kare_hizi=cfg.KAMERA_FPS)
        kaynak, ded, ciz_fn = k, k, k.ciz
        DURUM["motor"] = "HAILO NPU"
    else:
        from src.gorev.algilayici import Kamera
        kam = Kamera(args.kaynak, cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK,
                     cfg.KAMERA_FPS)
        if args.motor == "ncnn":
            from src.detect_ncnn import HedefDedektoruNCNN
            d = HedefDedektoruNCNN(cfg.NCNN_MODEL_DIZIN, conf=conf)
            DURUM["motor"] = "NCNN (CPU)"
        else:
            from src.detect import HedefDedektoru
            d = HedefDedektoru(cfg.PT_MODEL, conf=conf)
            DURUM["motor"] = "PyTorch"
        kaynak, ded, ciz_fn = kam, d, d.ciz

    threading.Thread(target=dongu, args=(kaynak, ded, ciz_fn), daemon=True).start()

    from flask import Flask, Response, jsonify
    app = Flask(__name__)
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    @app.route("/")
    def _k():
        return SAYFA

    @app.route("/durum")
    def _d():
        with _kilit:
            return jsonify(dict(DURUM))

    @app.route("/video")
    def _v():
        def uret():
            while True:
                with _kilit:
                    f = None if _kare["bgr"] is None else _kare["bgr"].copy()
                if f is None:
                    time.sleep(0.1)
                    continue
                h, w = f.shape[:2]
                if w > 960:
                    f = cv2.resize(f, (960, int(h * 960 / w)))
                ok, buf = cv2.imencode(".jpg", f,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok:
                    yield (b"--kare\r\nContent-Type: image/jpeg\r\n\r\n"
                           + buf.tobytes() + b"\r\n")
                time.sleep(1 / 12.0)
        return Response(uret(),
                        mimetype="multipart/x-mixed-replace; boundary=kare")

    print("=" * 62)
    print(f"  CANLI IZLEME  ->  http://{yerel_ip()}:{args.port}")
    print(f"  motor: {DURUM['motor']}   conf: {conf}")
    print("  Pixhawk'a BAGLANMAZ, hicbir komut GONDERMEZ.")
    print("=" * 62)
    app.run(host="0.0.0.0", port=args.port, debug=False,
            use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
