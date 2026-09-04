"""
ŞAFAK UAV - ARM-Optimize Dedektör (NCNN / Raspberry Pi 5)
=========================================================
NCNN, ARM-NEON için elden optimize edilmiş bir çıkarım motorudur; RPi5'te aynı model
onnxruntime'a göre belirgin daha hızlı çalışır (torch da gerekmez).

Çıktı arayüzü src/detect.py ve src/detect_onnx.py ile BİREBİR AYNIDIR: aynı Hedef
nesnesi, aynı isle() sözlüğü, aynı dx/dy sapma. Görev yazılımı için yer değiştirebilir.
Ön işleme (letterbox), son işleme (NMS + renk doğrulama + en iyi hedef seçimi) ve çizim
detect_onnx.py'den yeniden kullanılır — NCNN çıktısı ONNX ile aynı şekle sahip.

PC'de model üretimi:
    python tools/export_model.py --format ncnn        # models/safak_yolov8n_ncnn_model/

RPi5 kurulumu (torch YOK):
    sudo apt install -y python3-opencv python3-pip
    python3 -m venv ~/safak-venv && source ~/safak-venv/bin/activate
    pip install ncnn numpy

Kullanım:
    python src/detect_ncnn.py --kaynak 0 --goster
    python src/detect_ncnn.py --kaynak 0 --kaydet          # başsız (SSH) — video kaydeder
    python src/detect_ncnn.py --kaynak dataset/images/test --kaydet
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config  # noqa: E402
from detect_onnx import Hedef, _letterbox, ciz, son_isle  # noqa: E402,F401

import cv2  # noqa: E402
import numpy as np  # noqa: E402

NCNN_DIZIN = config.MODEL_DIZIN / "safak_yolov8n_ncnn_model"


class HedefDedektoruNCNN:
    def __init__(self, model_dizin=None, conf=None, iou=None, renk_dogrula=True, imgsz=None):
        import ncnn
        dizin = Path(model_dizin) if model_dizin else NCNN_DIZIN
        param = dizin / "model.ncnn.param"
        bin_ = dizin / "model.ncnn.bin"
        if not param.exists() or not bin_.exists():
            raise FileNotFoundError(
                f"NCNN modeli bulunamadi: {dizin}\n"
                f"PC'de uretin:  python tools/export_model.py --format ncnn")

        self.net = ncnn.Net()
        # Thread sınırı: 4 çekirdeği birden doyurmak RPi5'te ısı/akım tepesi yaratıp
        # sistemi termal korumaya sokuyor (bkz. config.CIKARIM_THREAD).
        self.net.opt.num_threads = config.CIKARIM_THREAD
        self.net.opt.use_vulkan_compute = False   # RPi5'te Vulkan yok; CPU/NEON
        self.net.load_param(str(param))
        self.net.load_model(str(bin_))

        self.imgsz = imgsz or config.IMGSZ_DAGITIM
        self.conf = conf if conf is not None else config.GUVEN_ESIGI
        self.iou = iou if iou is not None else config.IOU_ESIGI
        self.renk_dogrula = renk_dogrula

    def isle(self, bgr):
        import ncnn
        tuval, o, sol, ust = _letterbox(bgr, self.imgsz)

        # NCNN girişi from_pixels ile verilmeli. Ön-normalize edilmiş bir numpy dizisini
        # doğrudan ncnn.Mat'e vermek kanal bellek düzenini bozuyor (skorlar sıfırlanıyor,
        # hatta segfault). Model RGB [0,1] bekliyor; tuval OpenCV'den BGR geliyor.
        mat = ncnn.Mat.from_pixels(tuval, ncnn.Mat.PixelType.PIXEL_BGR2RGB,
                                   self.imgsz, self.imgsz)
        mat.substract_mean_normalize([], [1 / 255.0] * 3)

        with self.net.create_extractor() as ex:
            ex.input("in0", mat)
            _, cikti = ex.extract("out0")            # (4+nc, anchor)
        cikis = np.expand_dims(np.array(cikti), axis=0)   # -> (1, 4+nc, anchor)
        return son_isle(bgr, cikis, o, sol, ust, self.conf, self.iou, self.renk_dogrula)

    def ciz(self, bgr, sonuc):
        return ciz(bgr, sonuc)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kaynak", required=True, help="goruntu / klasor / video / '0' (webcam)")
    ap.add_argument("--model", default=None, help="NCNN model dizini")
    ap.add_argument("--imgsz", type=int, default=None,
                    help=f"cikarim cozunurlugu (varsayilan {config.IMGSZ_DAGITIM})")
    ap.add_argument("--conf", type=float, default=None)
    ap.add_argument("--kaydet", action="store_true")
    ap.add_argument("--goster", action="store_true", help="pencerede goster (ekran/VNC gerekir)")
    args = ap.parse_args()

    ded = HedefDedektoruNCNN(model_dizin=args.model, conf=args.conf, imgsz=args.imgsz)
    print(f"NCNN hazir | imgsz={ded.imgsz} | thread={config.CIKARIM_THREAD}")
    cikti = config.PROJE_KOK / "runs" / "tespit_ncnn"
    cikti.mkdir(parents=True, exist_ok=True)

    # --- Webcam / video ---
    if args.kaynak == "0" or Path(args.kaynak).suffix.lower() in {".mp4", ".avi", ".mov", ".mkv"}:
        cap = cv2.VideoCapture(0 if args.kaynak == "0" else args.kaynak)
        if not cap.isOpened():
            print("HATA: kamera/video acilamadi. Webcam icin --kaynak 1 deneyin.")
            return
        yazici = None
        try:
            while True:
                ok, kare = cap.read()
                if not ok:
                    break
                t = time.perf_counter()
                sonuc = ded.isle(kare)
                fps = 1.0 / max(1e-6, time.perf_counter() - t)
                gorsel = ded.ciz(kare, sonuc)
                cv2.putText(gorsel, f"{fps:.1f} FPS", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                if args.kaydet:
                    if yazici is None:
                        h, w = gorsel.shape[:2]
                        yazici = cv2.VideoWriter(str(cikti / "cikti.mp4"),
                                                 cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
                    yazici.write(gorsel)
                if args.goster:
                    cv2.imshow("SAFAK UAV (NCNN)", gorsel)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            if yazici:
                yazici.release()
                print(f"Video kaydedildi -> {cikti/'cikti.mp4'}")
            cv2.destroyAllWindows()
        return

    # --- Tek görüntü veya klasör ---
    yol = Path(args.kaynak)
    goruntuler = [yol] if yol.is_file() else sorted(
        p for p in yol.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"})
    print(f"{len(goruntuler)} goruntu isleniyor...")
    for g in goruntuler:
        bgr = cv2.imread(str(g))
        if bgr is None:
            continue
        t = time.perf_counter()
        sonuc = ded.isle(bgr)
        ms = (time.perf_counter() - t) * 1000
        for k in ("kirmizi_hedef", "mavi_hedef"):
            h = sonuc[k]
            durum = (f"guven={h.guven:.2f} sapma=({h.dx:+.2f},{h.dy:+.2f}) "
                     f"renk={h.renk_orani:.2f}" if h else "yok")
            print(f"  [{g.stem}] {k}: {durum}")
        print(f"      ({ms:.0f} ms/kare)")
        if args.kaydet:
            cv2.imwrite(str(cikti / g.name), ded.ciz(bgr, sonuc))
    if args.kaydet:
        print(f"Cikti -> {cikti}")


if __name__ == "__main__":
    main()
