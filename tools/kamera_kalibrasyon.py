"""
ŞAFAK UAV - Kamera Kalibrasyonu
================================
Geolokasyonun iki sistematik hata kaynağını ölçümle ortadan kaldırır:

  1. GÖRÜŞ AÇISI (HFOV) — şu an 66 derece VARSAYILIYOR (Camera Module 3
     standart lens katalog değeri). Yanlışsa tüm mesafeler aynı oranda
     kayar: %10 hatalı HFOV, hedefi 20 m'den bakarken 2 m yanlış yere koyar.
     Katalog değeri ayrıca kameranın hangi modda çalıştığına göre değişir
     (kırpılmış modda görüş açısı daralır). Bu yüzden ÖLÇMEK gerekir.

  2. MONTAJ YÖNÜ — kameranın üst kenarı gerçekten burna mı bakıyor? Yanlışsa
     hedef doğru mesafeye ama YANLIŞ YÖNE konur; 90 derece hata, hedefi tam
     yan tarafa taşır.

Her ikisi de yerde, brandayla, 10 dakikada ölçülür.

    python tools/kamera_kalibrasyon.py --kaynak picam --hfov
    python tools/kamera_kalibrasyon.py --kaynak picam --montaj
"""
import argparse
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from src.gorev import gorev_config as cfg
from src.gorev.algilayici import Kamera


def _dedektor():
    try:
        from src.detect_ncnn import HedefDedektoruNCNN
        return HedefDedektoruNCNN(cfg.NCNN_MODEL_DIZIN, conf=0.35)
    except Exception:
        from src.detect import HedefDedektoru
        return HedefDedektoru(cfg.PT_MODEL, conf=0.35)


def _hedef_bul(ded, kam, saniye, ad=""):
    """Belirtilen süre boyunca en güvenilir hedefi izler; ölçümleri döndürür."""
    t0 = time.time()
    olcumler = []
    son_ciz = None
    while time.time() - t0 < saniye:
        ok, kare = kam.oku()
        if not ok:
            continue
        s = ded.isle(kare)
        aday = None
        for k in ("kirmizi_hedef", "mavi_hedef"):
            h = s[k]
            if h and (aday is None or h.alan > aday.alan):
                aday = h
        if aday:
            x1, y1, x2, y2 = aday.kutu
            olcumler.append((aday.cx, aday.cy, x2 - x1, y2 - y1, aday.sinif_isim))
        son_ciz = ded.ciz(kare, s)
    if son_ciz is not None and ad:
        cfg.KANIT_DIZIN.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(cfg.KANIT_DIZIN / f"kalibrasyon_{ad}.jpg"), son_ciz)
    return olcumler


def hfov_olc(args):
    print("""
==================================================================
GORUS ACISI (HFOV) OLCUMU
==================================================================
Yapilacaklar:
  1. Brandayi DUZ bir zemine, kirissiz serin.
  2. Kamerayi brandanin TAM USTUNDE, mumkun oldugunca DIK asagi bakacak
     sekilde tutun (egiklik olcumu bozar).
  3. Kamera merceginden brandanin yuzeyine olan dikey mesafeyi METRE ile
     olcun. 2-3 m ideal: brandanin tamami kadraja girsin ama kucuk kalmasin.
  4. Branda kadrajin MERKEZINE yakin olsun (kenarda mercek bozulmasi artar).

Formul:  f = piksel_boyut * mesafe / gercek_boyut
         HFOV = 2 * atan( (kare_genisligi/2) / f )
""")
    try:
        gercek = float(input(">>> Brandanin gercek kenar uzunlugu (metre, orn 1.0): "))
        mesafe = float(input(">>> Kamera-branda dikey mesafesi (metre, orn 2.5): "))
    except (ValueError, EOFError):
        print("gecersiz giris")
        return
    input(">>> Hazir olunca ENTER (5 saniye olcum alinacak)...")

    kam = Kamera(args.kaynak, cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK,
                 cfg.KAMERA_FPS)
    ded = _dedektor()
    olcumler = _hedef_bul(ded, kam, 5.0, "hfov")
    kam.kapat()

    if len(olcumler) < 5:
        print(f"\n[HATA] yeterli tespit yok ({len(olcumler)} kare). Brandanin")
        print("       kadrajda ve iyi isikta oldugundan emin olun.")
        return

    G = cfg.KAMERA_GENISLIK
    # Kare hedefte genislik ve yukseklik ayni olmali; ikisinin medyanini al.
    # Bir kenar sistematik olarak buyukse kamera egiktir -> uyar.
    w = statistics.median(o[2] for o in olcumler)
    h = statistics.median(o[3] for o in olcumler)
    print(f"\n  {len(olcumler)} kare olculdu ({olcumler[0][4]})")
    print(f"  kutu genislik={w:.1f} px  yukseklik={h:.1f} px  oran={w/h:.3f}")
    if abs(w / h - 1.0) > 0.15:
        print("  !! UYARI: kutu kare degil, kamera egik duruyor ya da branda")
        print("     kirisik. Olcum guvenilmez, duzeltip tekrarlayin.")

    px = (w + h) / 2.0
    f = px * mesafe / gercek
    hfov = 2 * math.degrees(math.atan((G / 2) / f))

    print(f"\n  odak uzakligi f = {f:.1f} piksel")
    print(f"  OLCULEN HFOV    = {hfov:.1f} derece")
    print(f"  config'deki     = {cfg.KAMERA_HFOV_DERECE:.1f} derece")
    fark = abs(hfov - cfg.KAMERA_HFOV_DERECE) / cfg.KAMERA_HFOV_DERECE * 100
    print(f"  fark            = %{fark:.1f}")
    if fark > 5:
        print(f"\n  >>> gorev_config.py: KAMERA_HFOV_DERECE = {hfov:.1f}")
        print(f"      Bu duzeltilmezse 20 m'den bakarken hedef ~"
              f"{20*math.tan(math.radians(hfov/2))*fark/100:.2f} m yanlis yere konur.")
    else:
        print("\n  Katalog degeri dogrulandi, degistirmeye gerek yok.")


def montaj_olc(args):
    print("""
==================================================================
KAMERA MONTAJ YONU TESPITI
==================================================================
Mantik: Kamera asagi bakiyorken arac ILERI giderse, yerdeki bir nesne
kadrajda geriye kayar. Hangi yone kaydigi, kameranin govdeye gore
donusunu soyler:

    ASAGI kayarsa  -> montaj yaw =   0  (ust kenar burna bakiyor)
    SAGA  kayarsa  -> montaj yaw =  90
    YUKARI kayarsa -> montaj yaw = 180
    SOLA  kayarsa  -> montaj yaw = 270

Yapilacaklar:
  1. Brandayi yere serin, aracin kamerasini ustune dogru tutun (1.5-3 m).
  2. Branda kadrajin MERKEZINDE olsun.
  3. Olcum baslayinca araci, KENDI BURNUNUN baktigi yone dogru yaklasik
     yarim metre KAYDIRIN (dondurmeyin, oteleyin!).
""")
    input(">>> Branda merkezde, hazir olunca ENTER...")
    kam = Kamera(args.kaynak, cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK,
                 cfg.KAMERA_FPS)
    ded = _dedektor()

    print("\n  Baslangic konumu olculuyor (3 sn) — ARACI OYNATMAYIN...")
    once = _hedef_bul(ded, kam, 3.0, "montaj_once")
    if len(once) < 3:
        print("[HATA] branda bulunamadi")
        kam.kapat()
        return
    x0 = statistics.median(o[0] for o in once)
    y0 = statistics.median(o[1] for o in once)
    print(f"  baslangic: ({x0:.0f}, {y0:.0f})")

    print("\n  >>> SIMDI ARACI BURNUNUN YONUNE DOGRU KAYDIRIN <<<")
    print("  (6 saniyeniz var)")
    time.sleep(2.0)
    sonra = _hedef_bul(ded, kam, 4.0, "montaj_sonra")
    kam.kapat()
    if len(sonra) < 3:
        print("[HATA] hareket sonrasi branda bulunamadi (kadrajdan cikmis olabilir)")
        return
    x1 = statistics.median(o[0] for o in sonra)
    y1 = statistics.median(o[1] for o in sonra)

    dx, dy = x1 - x0, y1 - y0
    print(f"  bitis   : ({x1:.0f}, {y1:.0f})")
    print(f"  kayma   : dx={dx:+.0f} px, dy={dy:+.0f} px")

    if math.hypot(dx, dy) < 40:
        print("\n[HATA] kayma cok kucuk. Araci daha fazla kaydirip tekrarlayin.")
        return

    # Ileri hareket -> nesne kadrajda: yaw=0 icin +y, 90 icin +x, 180 -y, 270 -x
    aci = math.degrees(math.atan2(dx, dy)) % 360     # +y ekseni sifir
    secenekler = {0: 0, 90: 90, 180: 180, 270: 270}
    en_yakin = min(secenekler, key=lambda a: min(abs(aci - a), 360 - abs(aci - a)))
    sapma = min(abs(aci - en_yakin), 360 - abs(aci - en_yakin))

    print(f"\n  olculen aci     = {aci:.0f} derece")
    print(f"  EN YAKIN MONTAJ = {en_yakin} derece (sapma {sapma:.0f})")
    print(f"  config'deki     = {cfg.KAMERA_MONTAJ_YAW_DERECE:.0f} derece")
    if sapma > 25:
        print("  !! Sapma buyuk: arac duz kaydirilmamis ya da dondurulmus olabilir.")
        print("     Olcumu tekrarlayin.")
    if en_yakin != int(cfg.KAMERA_MONTAJ_YAW_DERECE):
        print(f"\n  >>> gorev_config.py: KAMERA_MONTAJ_YAW_DERECE = {en_yakin}")
        print("      Duzeltilmezse hedef DOGRU MESAFEYE ama YANLIS YONE konur.")
    else:
        print("\n  Montaj yonu dogrulandi.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kaynak", default="picam")
    ap.add_argument("--hfov", action="store_true", help="gorus acisini olc")
    ap.add_argument("--montaj", action="store_true", help="montaj yonunu tespit et")
    args = ap.parse_args()
    if not args.hfov and not args.montaj:
        ap.error("--hfov ya da --montaj verin")
    if args.hfov:
        hfov_olc(args)
    if args.montaj:
        montaj_olc(args)


if __name__ == "__main__":
    main()
