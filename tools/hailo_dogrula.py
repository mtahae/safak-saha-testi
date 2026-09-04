"""
ŞAFAK UAV - Hailo NPU Doğrulama (RPi5 + AI HAT+ üzerinde çalıştırılır)
=======================================================================
Hailo yolunu göreve almadan ÖNCE, sessizce yanlış sonuç üreten dört tuzağı
donanımda ölçer. Hiçbiri hata vermez; hepsi "çalışıyor gibi görünüp" yanlış
koordinat ya da sıfır tespit üretir.

    python tools/hailo_dogrula.py                    # tam denetim
    python tools/hailo_dogrula.py --sure 30          # daha uzun FPS olcumu
    python tools/hailo_dogrula.py --kaynak /dev/video0

DENETLENEN DÖRT ŞEY
-------------------
1. BORU HATTI  : arka planda çalışıyor mu, kare üretiyor mu
2. ETİKETLER   : "kirmizi_hedef"/"mavi_hedef" mi geliyor, yoksa COCO mu
                 (--labels-json yüklenmediyse "person"/"bicycle" gelir)
3. KIRPMA/FOV  : boru hattının karesi picamera2'nin tam-FOV karesiyle aynı
                 sahneyi mi gösteriyor? Kırpıyorsa KAMERA_HFOV_DERECE yanlış
                 kalır ve tüm GPS projeksiyonu sistematik kayar
4. HIZ         : kare/saniye — NCNN'e göre kazanç var mı

3. adım GÖZLE karşılaştırma gerektirir; araç iki kareyi yan yana kaydeder.
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import cv2
import numpy as np

from src.gorev import gorev_config as cfg

CIKTI = cfg.PROJE_KOK / "runs" / "hailo_dogrula"


def basli(s):
    print("\n" + "=" * 70)
    print(s)
    print("=" * 70)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hef", default=None)
    ap.add_argument("--etiket", default=None)
    ap.add_argument("--kaynak", default=cfg.HAILO_KAYNAK)
    ap.add_argument("--sure", type=float, default=15.0, help="FPS olcum suresi (sn)")
    ap.add_argument("--fov-atla", action="store_true",
                    help="picamera2 karsilastirmasini atla")
    args = ap.parse_args()

    CIKTI.mkdir(parents=True, exist_ok=True)
    gecti, kaldi = [], []

    # ---------------------------------------------------------------- 1
    basli("1) BORU HATTI")
    from src.detect_hailo import HailoKaynak, ETIKET_ESLEME
    t0 = time.time()
    kaynak = HailoKaynak(args.hef or cfg.HAILO_HEF,
                         args.etiket or cfg.HAILO_ETIKET_JSON,
                         kaynak=args.kaynak, conf=0.10,   # etiketleri gormek icin dusuk
                         kare_hizi=cfg.KAMERA_FPS)
    print(f"  boru hatti {time.time()-t0:.1f} sn'de acildi")
    ok, kare = kaynak.oku()
    if not ok:
        print("  !! kare gelmedi")
        kaldi.append("boru hatti kare uretmiyor")
        return _bitir(gecti, kaldi)
    H, W = kare.shape[:2]
    print(f"  boru hatti kare boyutu : {W}x{H}  (en/boy {W/H:.3f})")
    gecti.append("boru hatti calisiyor")

    # ---------------------------------------------------------------- 2 + 4
    basli(f"2) ETIKETLER ve 4) HIZ  ({args.sure:.0f} saniye olculuyor)")
    print("  >> Kameraya KIRMIZI ve MAVI brandayi gosterin <<")
    t0 = time.time()
    bas_kare = kaynak.kare_sayisi
    gorulen = {}
    ornek = None
    while time.time() - t0 < args.sure:
        ok, k = kaynak.oku()
        if not ok:
            time.sleep(0.05)
            continue
        s = kaynak.isle()
        for h in s["tum"]:
            g = gorulen.setdefault(h.sinif_isim, {"n": 0, "guven": 0.0, "renk": 0.0})
            g["n"] += 1
            g["guven"] = max(g["guven"], h.guven)
            g["renk"] = max(g["renk"], h.renk_orani)
        if s["tum"] and ornek is None:
            ornek = kaynak.ciz(k, s)
        time.sleep(0.02)
    gecen = time.time() - t0
    fps = (kaynak.kare_sayisi - bas_kare) / gecen
    print(f"\n  BORU HATTI HIZI: {fps:.1f} FPS")
    if fps >= 15:
        gecti.append(f"hiz {fps:.1f} FPS")
    else:
        kaldi.append(f"hiz dusuk ({fps:.1f} FPS) -- NCNN ile karsilastirin")

    print("\n  Taninan tespitler (renk dogrulamasindan GECENLER):")
    if gorulen:
        for ad, g in sorted(gorulen.items()):
            print(f"    {ad:<16} n={g['n']:<5} en_yuksek_guven={g['guven']:.2f} "
                  f"renk_orani={g['renk']:.2f}")
        gecti.append("etiketler dogru esleniyor")
    else:
        print("    (hicbiri)")

    if kaynak.bilinmeyen_etiket:
        print("\n  !! BILINMEYEN ETIKETLER (labels-json YUKLENMEMIS):")
        for ad, n in sorted(kaynak.bilinmeyen_etiket.items(),
                            key=lambda x: -x[1])[:10]:
            print(f"    '{ad}' x{n}")
        print("\n  Bu, modelin COCO siniflarini dondurdugu anlamina gelir.")
        print("  Kontrol edin: models/safak_etiketler.json okunuyor mu?")
        print("  Etiket sirasi su an: " + str(_etiketleri_oku(
            args.etiket or cfg.HAILO_ETIKET_JSON)))
        print("  Sinif KAYMASI ihtimali: listedeki 'unlabeled' dolgusunu")
        print("  KALDIRIP tekrar deneyin (bu dolgu bazi surumlerde gerekli,")
        print("  bazilarinda fazladir; ikisi de sessizce yanlis sonuc verir).")
        kaldi.append("bilinmeyen etiket geldi (labels-json)")
    elif not gorulen:
        kaldi.append("hic tespit alinmadi -- branda gosterildi mi?")

    if ornek is not None:
        yol = CIKTI / "tespit_ornegi.jpg"
        cv2.imwrite(str(yol), ornek)
        print(f"\n  ornek tespit karesi -> {yol}")

    # ---------------------------------------------------------------- 3
    basli("3) KIRPMA / GORUS ACISI")
    hailo_kare = kare.copy()
    kaynak.kapat()
    time.sleep(2.0)          # kamera serbest kalsin

    if args.fov_atla:
        print("  atlandi (--fov-atla)")
    else:
        try:
            from src.gorev.algilayici import Kamera
            kam = Kamera("picam", cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK,
                         cfg.KAMERA_FPS)
            time.sleep(1.5)
            ok2, tam = kam.oku()
            kam.kapat()
            if ok2:
                _fov_karsilastir(hailo_kare, tam, gecti, kaldi)
            else:
                kaldi.append("picamera2 karesi alinamadi")
        except Exception as e:
            print(f"  picamera2 acilamadi: {e}")
            kaldi.append("FOV karsilastirmasi yapilamadi")

    _bitir(gecti, kaldi)


def _etiketleri_oku(yol):
    import json
    try:
        return json.loads(Path(yol).read_text(encoding="utf-8")).get("labels")
    except Exception as e:
        return f"okunamadi: {e}"


def _fov_karsilastir(hailo_kare, tam_kare, gecti, kaldi):
    hH, hW = hailo_kare.shape[:2]
    tH, tW = tam_kare.shape[:2]
    print(f"  Hailo boru hatti : {hW}x{hH}  en/boy {hW/hH:.3f}")
    print(f"  picamera2 tam FOV: {tW}x{tH}  en/boy {tW/tH:.3f}")

    if abs(hW / hH - tW / tH) > 0.02:
        print("\n  !! EN/BOY ORANLARI FARKLI. Iki ihtimal var ve ikisi de")
        print("     geolokasyonu SESSIZCE bozar:")
        print()
        print("     (a) KIRPMA -- boru hatti kenarlardan kesiyor. O zaman gercek")
        print("         yatay gorus acisi 66 dereceden dardir:")
        print("            yeni_HFOV = 2*atan( tan(66/2) * (kirpilan/tam) )")
        print("         gorev_config.KAMERA_HFOV_DERECE'yi duzeltin.")
        print()
        print("     (b) ANAMORFIK GERME -- boru hatti kareyi en/boy oranini")
        print("         koruMADAN kareye gerdi. O zaman dikey olcek yataydan")
        print("         farklidir; tek odak uzakligi kullanmak hedefi sistematik")
        print("         olarak yanlis yere koyar. gorev_config.KAMERA_VFOV_DERECE:")
        # Germe varsayimi: yatay FOV korunuyor, dikey FOV tam sensorunki
        vfov_tam = 2 * math.degrees(math.atan(
            math.tan(math.radians(cfg.KAMERA_HFOV_DERECE) / 2.0) * (tH / tW)))
        print(f"            KAMERA_VFOV_DERECE = {vfov_tam:.1f}")
        print(f"         (tam FOV dikey acisi; su an None = kare piksel varsayimi)")
        print()
        print("     HANGISI oldugunu asagidaki goruntuden GOZLE ayirt edin:")
        print("       kenarlardan bir sey EKSIK  -> (a) kirpma")
        print("       her sey var ama BASIK/UZUN -> (b) germe")
        kaldi.append("en/boy orani farkli -- kirpma veya germe")
    else:
        print("\n  en/boy oranlari uyusuyor (oranli olcekleme, kirpma/germe yok)")
        print("  -> KAMERA_VFOV_DERECE = None (kare piksel) dogru varsayim")
        gecti.append("en/boy orani tutarli")

    o = 640 / hW
    a = cv2.resize(hailo_kare, (640, int(hH * o)))
    o = 640 / tW
    b = cv2.resize(tam_kare, (640, int(tH * o)))
    yuk = max(a.shape[0], b.shape[0])
    tuval = np.zeros((yuk + 30, 1280, 3), np.uint8)
    tuval[30:30 + a.shape[0], :640] = a
    tuval[30:30 + b.shape[0], 640:] = b
    cv2.putText(tuval, "HAILO boru hatti", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(tuval, "picamera2 TAM FOV", (650, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    yol = CIKTI / "fov_karsilastirma.jpg"
    cv2.imwrite(str(yol), tuval)
    print(f"\n  -> {yol}")
    print("     GOZLE BAKIN: iki kare AYNI sahneyi mi gosteriyor?")
    print("     Hailo karesinde kenarlardan bir sey EKSIKSE boru hatti kirpiyor;")
    print("     bu durumda geolokasyon sistematik olarak sapar.")


def _bitir(gecti, kaldi):
    basli("SONUC")
    for s in gecti:
        print(f"  [OK]    {s}")
    for s in kaldi:
        print(f"  [SORUN] {s}")
    if kaldi:
        print("\n  Hailo yolu HENUZ goreve alinmamali. NCNN ile ucun:")
        print("    python -m src.gorev.gorev2 --kaynak picam")
    else:
        print("\n  Hailo yolu kullanilabilir:")
        print("    python -m src.gorev.gorev2 --motor hailo --prova")
    return 1 if kaldi else 0


if __name__ == "__main__":
    sys.exit(main() or 0)
