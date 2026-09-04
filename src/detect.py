"""
ŞAFAK UAV - Gerçek-Zamanlı Hedef Dedektör Modülü (İP6)
======================================================
Eğitilmiş YOLOv8 modelini kullanarak bir kareden kırmızı (1x1) ve mavi (2x2)
hedefleri tespit eder, renk doğrulamasıyla aldatıcıları eler, her renk için EN İYİ
hedefi seçer ve görev yazılımının kullanacağı konumlanma bilgisini üretir:

    sinif, piksel merkez (cx, cy), kare merkezine göre normalize sapma (dx, dy),
    alan, güven, renk oranı.

dx, dy in [-1, 1]:  dx>0 hedef merkezin SAĞINDA, dy>0 hedef merkezin ALTINDA.
Görev durum makinesi bu sapmayı sıfıra indirerek İHA'yı hedefin tam üstüne getirir;
sapma eşik altına inince servo bırakma tetiklenir.

İki kullanım:
  1) Kütüphane:  HedefDedektoru sınıfı (modeli bir kez yükle, kare kare çağır)
  2) CLI:        python src/detect.py --kaynak foto/DJI_0540.JPG --kaydet

CLI kaynak: tek görüntü, klasör, video dosyası veya "0" (webcam).

Not: Yer istasyonuna canlı kanıt için annotated kareler sonraki aşamada GStreamer UDP
ile yayınlanacaktır (donanım/RPi tarafında). Bu modül donanımdan bağımsızdır.
"""
import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config  # noqa: E402
import color_verify  # noqa: E402

import cv2  # noqa: E402

CIZIM_RENK = {config.KIRMIZI: (0, 0, 255), config.MAVI: (255, 150, 0)}  # BGR


@dataclass
class Hedef:
    sinif_id: int
    sinif_isim: str
    guven: float
    kutu: tuple        # (x1, y1, x2, y2) piksel
    cx: float          # piksel merkez x
    cy: float          # piksel merkez y
    dx: float          # normalize sapma x [-1,1]
    dy: float          # normalize sapma y [-1,1]
    alan: int          # piksel alan
    renk_orani: float


class HedefDedektoru:
    def __init__(self, model_yolu=None, conf=None, iou=None, renk_dogrula=True, cihaz=None):
        from ultralytics import YOLO
        import torch
        if model_yolu is None:
            model_yolu = config.MODEL_DIZIN / "safak_yolov8n.pt"
        self.model = YOLO(str(model_yolu))
        self.conf = conf if conf is not None else config.GUVEN_ESIGI
        self.iou = iou if iou is not None else config.IOU_ESIGI
        self.renk_dogrula = renk_dogrula
        if cihaz is None:
            cihaz = 0 if torch.cuda.is_available() else "cpu"
        self.cihaz = cihaz

    def isle(self, bgr):
        """Bir BGR kareyi işler; her renk için en iyi hedefi ve tüm tespitleri döndürür."""
        H, W = bgr.shape[:2]
        mx, my = W / 2.0, H / 2.0
        sonuc = self.model.predict(bgr, conf=self.conf, iou=self.iou, imgsz=config.IMGSZ,
                                   device=self.cihaz, verbose=False)[0]

        tum = []
        for kutu in sonuc.boxes:
            sid = int(kutu.cls[0])
            guven = float(kutu.conf[0])
            x1, y1, x2, y2 = [float(v) for v in kutu.xyxy[0]]
            roran = color_verify.renk_orani(bgr, (x1, y1, x2, y2), sid)
            if self.renk_dogrula and roran < config.RENK_DOGRULAMA_MIN_ORAN:
                continue  # aldatıcı (renk eşleşmiyor) -> ele
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            tum.append(Hedef(
                sinif_id=sid,
                sinif_isim=config.SINIF_ISIMLERI[sid],
                guven=guven,
                kutu=(x1, y1, x2, y2),
                cx=cx, cy=cy,
                dx=(cx - mx) / mx,
                dy=(cy - my) / my,
                alan=int((x2 - x1) * (y2 - y1)),
                renk_orani=roran,
            ))

        # Her renk için EN İYİ: en yüksek güven
        en_iyi = {config.KIRMIZI: None, config.MAVI: None}
        for h in tum:
            mevcut = en_iyi[h.sinif_id]
            if mevcut is None or h.guven > mevcut.guven:
                en_iyi[h.sinif_id] = h

        return {
            "kirmizi_hedef": en_iyi[config.KIRMIZI],
            "mavi_hedef": en_iyi[config.MAVI],
            "tum": tum,
            "kare_boyut": (W, H),
        }

    def ciz(self, bgr, sonuc):
        """Tespitleri ve seçilen en iyi hedeflerin sapma okunu kare üzerine çizer."""
        out = bgr.copy()
        H, W = out.shape[:2]
        mx, my = W // 2, H // 2
        # kare merkezi nişangahı
        cv2.drawMarker(out, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)

        secilenler = {id(sonuc[k]) for k in ("kirmizi_hedef", "mavi_hedef") if sonuc[k]}
        for h in sonuc["tum"]:
            x1, y1, x2, y2 = [int(v) for v in h.kutu]
            renk = CIZIM_RENK[h.sinif_id]
            secili = id(h) in secilenler
            kalin = 4 if secili else 2
            cv2.rectangle(out, (x1, y1), (x2, y2), renk, kalin)
            etiket = f"{h.sinif_isim} {h.guven:.2f} r={h.renk_orani:.2f}"
            cv2.putText(out, etiket, (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, renk, 2)
            if secili:
                cv2.circle(out, (int(h.cx), int(h.cy)), 6, renk, -1)
                cv2.arrowedLine(out, (mx, my), (int(h.cx), int(h.cy)), renk, 2, tipLength=0.03)
                cv2.putText(out, f"dx={h.dx:+.2f} dy={h.dy:+.2f}", (x1, y2 + 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, renk, 2)
        return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _pencerede_goster(ad, gorsel, maks_genislik=1280, maks_yukseklik=800):
    """imshow penceresi ekrana sığsın diye görüntüyü sadece GÖSTERİM için küçültür
    (kaydedilen dosya orijinal çözünürlükte kalır)."""
    h, w = gorsel.shape[:2]
    olcek = min(maks_genislik / w, maks_yukseklik / h, 1.0)
    if olcek < 1.0:
        gorsel = cv2.resize(gorsel, (int(w * olcek), int(h * olcek)))
    cv2.imshow(ad, gorsel)


def _goruntu_mu(p: Path):
    return p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}


def _video_mu(p: Path):
    return p.suffix.lower() in {".mp4", ".avi", ".mov", ".mkv"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kaynak", required=True, help="görüntü / klasör / video / '0' (webcam)")
    ap.add_argument("--model", default=None, help="ağırlık yolu (varsayılan models/safak_yolov8n.pt)")
    ap.add_argument("--conf", type=float, default=None)
    ap.add_argument("--kaydet", action="store_true", help="annotated çıktıyı kaydet")
    ap.add_argument("--goster", action="store_true", help="pencerede göster")
    ap.add_argument("--renk-dogrulama-yok", action="store_true")
    args = ap.parse_args()

    ded = HedefDedektoru(model_yolu=args.model, conf=args.conf,
                         renk_dogrula=not args.renk_dogrulama_yok)
    cikti_dizin = config.PROJE_KOK / "runs" / "tespit"
    cikti_dizin.mkdir(parents=True, exist_ok=True)

    def _ozet(sonuc, ad):
        for k in ("kirmizi_hedef", "mavi_hedef"):
            h = sonuc[k]
            if h:
                print(f"  [{ad}] {k}: guven={h.guven:.2f} merkez=({h.cx:.0f},{h.cy:.0f}) "
                      f"sapma=({h.dx:+.2f},{h.dy:+.2f}) renk={h.renk_orani:.2f}")
            else:
                print(f"  [{ad}] {k}: yok")

    # Webcam / video
    if args.kaynak == "0" or _video_mu(Path(args.kaynak)):
        kaynak = 0 if args.kaynak == "0" else args.kaynak
        cap = cv2.VideoCapture(kaynak)
        kaynak_fps = cap.get(cv2.CAP_PROP_FPS) or 25.0  # kaynak 0/webcam icin FPS bilinmeyebilir
        yazici = None
        kare_no = 0
        while True:
            ok, kare = cap.read()
            if not ok:
                break
            sonuc = ded.isle(kare)
            ciz = ded.ciz(kare, sonuc)
            if args.kaydet:
                if yazici is None:
                    h, w = ciz.shape[:2]
                    yol = cikti_dizin / "cikti.mp4"
                    yazici = cv2.VideoWriter(str(yol), cv2.VideoWriter_fourcc(*"mp4v"),
                                             kaynak_fps, (w, h))
                yazici.write(ciz)
            if args.goster:
                _pencerede_goster("SAFAK UAV", ciz)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            kare_no += 1
        cap.release()
        if yazici:
            yazici.release()
            print(f"Video kaydedildi: {cikti_dizin/'cikti.mp4'}")
        cv2.destroyAllWindows()
        return

    # Tek görüntü veya klasör
    yol = Path(args.kaynak)
    goruntuler = [yol] if yol.is_file() else sorted(
        [p for p in yol.iterdir() if _goruntu_mu(p)])
    print(f"{len(goruntuler)} goruntu isleniyor...")
    for g in goruntuler:
        bgr = cv2.imread(str(g))
        if bgr is None:
            continue
        sonuc = ded.isle(bgr)
        _ozet(sonuc, g.stem)
        if args.kaydet:
            ciz = ded.ciz(bgr, sonuc)
            cv2.imwrite(str(cikti_dizin / g.name), ciz)
        if args.goster:
            _pencerede_goster("SAFAK UAV", ded.ciz(bgr, sonuc))
            cv2.waitKey(0)
    if args.kaydet:
        print(f"Cikti -> {cikti_dizin}")
    if args.goster:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
