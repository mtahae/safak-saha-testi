"""
ŞAFAK UAV - Hafif ONNX Dedektör (RPi5 / torch'suz)  [Aşama A]
============================================================
Eğitilmiş modeli ONNX olarak, yalnızca onnxruntime + OpenCV ile çalıştırır.
Raspberry Pi 5'te torch kurmaya gerek kalmaz (hafif ve hızlı kurulum).
Çıktı, src/detect.py ile AYNIDIR: her renk için en iyi hedef + merkez + sapma
(dx,dy) + renk doğrulama. Böylece görev yazılımı için yer değiştirebilir (drop-in).

YOLOv8 ONNX çıktısı: [1, 4+nc, 8400] -> satır 0-3 kutu (cx,cy,w,h @640), 4.. sınıf skorları.

Kullanım (RPi'de USB webcam):
    python src/detect_onnx.py --kaynak 0 --goster
    python src/detect_onnx.py --kaynak foto/DJI_0540.JPG --kaydet
    python src/detect_onnx.py --kaynak 0 --model models/safak_yolov8n.onnx
"""
import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config  # noqa: E402
import color_verify  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402

CIZIM_RENK = {config.KIRMIZI: (0, 0, 255), config.MAVI: (255, 150, 0)}


@dataclass
class Hedef:
    sinif_id: int
    sinif_isim: str
    guven: float
    kutu: tuple
    cx: float
    cy: float
    dx: float
    dy: float
    alan: int
    renk_orani: float


def _letterbox(bgr, boyut=640):
    """Görüntüyü en-boy oranını koruyarak boyut x boyut kareye yerleştirir (gri dolgu)."""
    h, w = bgr.shape[:2]
    o = min(boyut / h, boyut / w)
    yh, yw = int(round(h * o)), int(round(w * o))
    kucuk = cv2.resize(bgr, (yw, yh), interpolation=cv2.INTER_LINEAR)
    tuval = np.full((boyut, boyut, 3), 114, np.uint8)
    ust, sol = (boyut - yh) // 2, (boyut - yw) // 2
    tuval[ust:ust + yh, sol:sol + yw] = kucuk
    return tuval, o, sol, ust


def on_isle(bgr, imgsz):
    """BGR kare -> ağ girişi (1,3,imgsz,imgsz float32 RGB [0,1]) + geri dönüşüm katsayıları."""
    tuval, o, sol, ust = _letterbox(bgr, imgsz)
    x = tuval[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return x, o, sol, ust


def son_isle(bgr, cikis, o, sol, ust, conf, iou, renk_dogrula):
    """Ham YOLOv8 çıktısını (1,4+nc,8400) Hedef listesine + en-iyi seçimine çevirir.

    ONNX ve NCNN aynı çıktı şekline sahip olduğu için iki dedektör de bunu kullanır.
    """
    H, W = bgr.shape[:2]
    mx, my = W / 2.0, H / 2.0

    cikis = cikis[0].T  # [8400, 4+nc]
    kutular_xywh = cikis[:, :4]
    skorlar = cikis[:, 4:]
    siniflar = skorlar.argmax(1)
    guvenler = skorlar.max(1)
    secim = guvenler >= conf
    kutular_xywh, siniflar, guvenler = kutular_xywh[secim], siniflar[secim], guvenler[secim]

    # cxcywh(@imgsz, letterbox) -> xyxy(orijinal görüntü)
    kutular = []
    for (cx, cy, w, h) in kutular_xywh:
        kutular.append([(cx - w / 2 - sol) / o, (cy - h / 2 - ust) / o,
                        (cx + w / 2 - sol) / o, (cy + h / 2 - ust) / o])
    kutular = np.array(kutular, dtype=np.float32) if kutular else np.zeros((0, 4), np.float32)

    # Sınıf bazında NMS (OpenCV)
    tum = []
    for sid in (config.KIRMIZI, config.MAVI):
        m = siniflar == sid
        if not m.any():
            continue
        k = kutular[m]
        g = guvenler[m]
        kutu_wh = [[int(a[0]), int(a[1]), int(a[2] - a[0]), int(a[3] - a[1])] for a in k]
        idx = cv2.dnn.NMSBoxes(kutu_wh, g.tolist(), conf, iou)
        if len(idx) == 0:
            continue
        for i in np.array(idx).flatten():
            x1, y1, x2, y2 = [float(v) for v in k[i]]
            roran = color_verify.renk_orani(bgr, (x1, y1, x2, y2), sid)
            if renk_dogrula and roran < config.RENK_DOGRULAMA_MIN_ORAN:
                continue  # aldatıcı (renk eşleşmiyor) -> ele
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            tum.append(Hedef(sid, config.SINIF_ISIMLERI[sid], float(g[i]),
                             (x1, y1, x2, y2), cx, cy,
                             (cx - mx) / mx, (cy - my) / my,
                             int((x2 - x1) * (y2 - y1)), roran))

    en_iyi = {config.KIRMIZI: None, config.MAVI: None}
    for h in tum:
        if en_iyi[h.sinif_id] is None or h.guven > en_iyi[h.sinif_id].guven:
            en_iyi[h.sinif_id] = h
    return {"kirmizi_hedef": en_iyi[config.KIRMIZI], "mavi_hedef": en_iyi[config.MAVI],
            "tum": tum, "kare_boyut": (W, H)}


def ciz(bgr, sonuc):
    """Tespitleri + seçilen hedeflerin sapma okunu çizer (ONNX ve NCNN ortak)."""
    out = bgr.copy()
    H, W = out.shape[:2]
    mx, my = W // 2, H // 2
    cv2.drawMarker(out, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
    secilen = {id(sonuc[k]) for k in ("kirmizi_hedef", "mavi_hedef") if sonuc[k]}
    for h in sonuc["tum"]:
        x1, y1, x2, y2 = [int(v) for v in h.kutu]
        renk = CIZIM_RENK[h.sinif_id]
        sec = id(h) in secilen
        cv2.rectangle(out, (x1, y1), (x2, y2), renk, 4 if sec else 2)
        cv2.putText(out, f"{h.sinif_isim} {h.guven:.2f}", (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, renk, 2)
        if sec:
            cv2.arrowedLine(out, (mx, my), (int(h.cx), int(h.cy)), renk, 2, tipLength=0.03)
            cv2.putText(out, f"dx={h.dx:+.2f} dy={h.dy:+.2f}", (x1, y2 + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, renk, 2)
    return out


class HedefDedektoruONNX:
    def __init__(self, model_yolu=None, conf=None, iou=None, renk_dogrula=True, imgsz=None):
        import onnxruntime as ort
        if model_yolu is None:
            model_yolu = config.MODEL_DIZIN / "safak_yolov8n.onnx"
        # Thread sınırı: 4 çekirdeği birden doyurmak RPi5'te ısı/akım tepesi yaratıp
        # sistemi termal korumaya sokuyor (bkz. config.CIKARIM_THREAD).
        se = ort.SessionOptions()
        se.intra_op_num_threads = config.CIKARIM_THREAD
        se.inter_op_num_threads = 1
        self.oturum = ort.InferenceSession(str(model_yolu), sess_options=se,
                                           providers=["CPUExecutionProvider"])
        self.giris_ad = self.oturum.get_inputs()[0].name
        # ONNX girişi sabit boyutlu: modelin kendi beyan ettiği boyutu kullan.
        giris_sekil = self.oturum.get_inputs()[0].shape
        self.imgsz = imgsz or (giris_sekil[2] if isinstance(giris_sekil[2], int)
                               else config.IMGSZ_DAGITIM)
        self.conf = conf if conf is not None else config.GUVEN_ESIGI
        self.iou = iou if iou is not None else config.IOU_ESIGI
        self.renk_dogrula = renk_dogrula

    def isle(self, bgr):
        x, o, sol, ust = on_isle(bgr, self.imgsz)
        cikis = self.oturum.run(None, {self.giris_ad: x})[0]  # [1, 4+nc, 8400]
        return son_isle(bgr, cikis, o, sol, ust, self.conf, self.iou, self.renk_dogrula)

    def ciz(self, bgr, sonuc):
        return ciz(bgr, sonuc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kaynak", required=True, help="görüntü / video / '0' (webcam)")
    ap.add_argument("--model", default=None)
    ap.add_argument("--conf", type=float, default=None)
    ap.add_argument("--kaydet", action="store_true")
    ap.add_argument("--goster", action="store_true")
    args = ap.parse_args()

    ded = HedefDedektoruONNX(model_yolu=args.model, conf=args.conf)
    cikti = config.PROJE_KOK / "runs" / "tespit_onnx"
    cikti.mkdir(parents=True, exist_ok=True)

    if args.kaynak == "0" or Path(args.kaynak).suffix.lower() in {".mp4", ".avi", ".mov", ".mkv"}:
        cap = cv2.VideoCapture(0 if args.kaynak == "0" else args.kaynak)
        while True:
            ok, kare = cap.read()
            if not ok:
                break
            t = time.time()
            sonuc = ded.isle(kare)
            fps = 1.0 / max(1e-6, time.time() - t)
            ciz = ded.ciz(kare, sonuc)
            cv2.putText(ciz, f"{fps:.1f} FPS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            if args.goster:
                cv2.imshow("SAFAK UAV (ONNX)", ciz)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        cap.release()
        cv2.destroyAllWindows()
        return

    yol = Path(args.kaynak)
    goruntuler = [yol] if yol.is_file() else sorted(yol.glob("*.JPG"))
    for g in goruntuler:
        bgr = cv2.imread(str(g))
        if bgr is None:
            continue
        t = time.time()
        sonuc = ded.isle(bgr)
        dt = (time.time() - t) * 1000
        for k in ("kirmizi_hedef", "mavi_hedef"):
            h = sonuc[k]
            print(f"  [{g.stem}] {k}: " + (f"guven={h.guven:.2f} sapma=({h.dx:+.2f},{h.dy:+.2f}) "
                  f"renk={h.renk_orani:.2f}" if h else "yok") + (f"   ({dt:.0f}ms)" if k == "kirmizi_hedef" else ""))
        if args.kaydet:
            cv2.imwrite(str(cikti / g.name), ded.ciz(bgr, sonuc))
    if args.kaydet:
        print(f"Cikti -> {cikti}")


if __name__ == "__main__":
    main()
