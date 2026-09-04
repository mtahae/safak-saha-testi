"""
ŞAFAK UAV - Görev 2 Rota Üreteci
=================================
Sahada verilen koordinatlardan, Mission Planner'a doğrudan yüklenebilen bir
AUTO görevi (.waypoints) üretir.

NEDEN: Yarışmada kurulum 8 dakika. O telaşta Mission Planner'da elle waypoint
tıklamak hem yavaş hem hatalı — ve tarama şeritlerinin aralığını gözle doğru
tutturmak mümkün değil. Bu araç şerit aralığını kameranın görüş açısından
HESAPLAR, tarama alanının tamamının görüntülendiğini garantiler ve tespit
kapısının hangi waypoint numarası olduğunu söyler.

Şartname kısıtları koda gömülü:
  - Hedef tespiti 2. direk DIŞTAN alındıktan SONRA yapılmalı
  - Tarama irtifası 20 m (ölçümle belirlendi; 28 m'de model hedefi göremiyor)
  - Uçuş süresi en fazla 10 dakika

    python tools/rota_uret.py \\
        --direk1 39.933400,32.859700 \\
        --direk2 39.934500,32.858200 \\
        --kalkis 39.933100,32.860200 \\
        --tarama 39.934100,32.859400 39.934600,32.858600
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.gorev import geo, gorev_config as cfg

# MAVLink komutları
NAV_WAYPOINT = 16
NAV_TAKEOFF = 22
NAV_LAND = 21
FRAME_REL_ALT = 3


def koordinat(s):
    try:
        lat, lon = (float(x) for x in s.replace(" ", "").split(","))
        return lat, lon
    except Exception:
        raise argparse.ArgumentTypeError(f"gecersiz koordinat: {s} (lat,lon bekleniyor)")


def yon_vektoru(a, b):
    """a'dan b'ye birim kuzey/doğu vektörü ve mesafe."""
    kn, de = geo.kuzey_dogu_farki(a[0], a[1], b[0], b[1])
    m = math.hypot(kn, de)
    if m < 1e-6:
        return (0.0, 0.0), 0.0
    return (kn / m, de / m), m


def oteleyerek(nokta, kuzey, dogu):
    return geo.oteleme_uygula(nokta[0], nokta[1], kuzey, dogu)


def serit_genisligi(irtifa, kam):
    """Kameranın o irtifadaki yer izdüşüm genişliği (metre)."""
    f = kam.odak_piksel(cfg.KAMERA_GENISLIK)
    return 2.0 * irtifa * (cfg.KAMERA_GENISLIK / 2.0) / f


def tarama_seritleri(kose1, kose2, irtifa, kam, ortusme=0.25):
    """Dikdörtgen tarama alanını kaplayan ileri-geri şeritler üretir.

    Şerit aralığı kameranın görüş açısından hesaplanır; %25 örtüşme bırakılır
    çünkü rüzgâr ve waypoint toleransı aracı rotadan birkaç metre kaydırır ve
    örtüşme olmazsa şeritler arasında KÖR ŞERİT kalır.
    """
    # Alanı yerel kuzey/doğu metre düzlemine taşı
    kn, de = geo.kuzey_dogu_farki(kose1[0], kose1[1], kose2[0], kose2[1])
    uzun_kuzey = abs(kn) >= abs(de)

    genislik = serit_genisligi(irtifa, kam)
    aralik = genislik * (1.0 - ortusme)

    if uzun_kuzey:
        # Şeritler kuzey-güney uzanır, doğuya doğru kayar
        boy, en = kn, de
    else:
        boy, en = de, kn

    n = max(1, int(math.ceil(abs(en) / aralik)))
    # Şeritleri alana eşit dağıt; ilk ve son şerit kenardan yarım aralık içeride
    noktalar = []
    for i in range(n):
        t = (i + 0.5) / n
        yanal = en * t
        ileri_bas = 0.0 if i % 2 == 0 else boy
        ileri_son = boy if i % 2 == 0 else 0.0
        for ileri in (ileri_bas, ileri_son):
            if uzun_kuzey:
                noktalar.append(oteleyerek(kose1, ileri, yanal))
            else:
                noktalar.append(oteleyerek(kose1, yanal, ileri))
    return noktalar, n, genislik, aralik


def satir(idx, komut, lat, lon, alt, p1=0, current=0, frame=FRAME_REL_ALT):
    return (f"{idx}\t{current}\t{frame}\t{komut}\t{p1:.6f}\t0.000000\t0.000000\t"
            f"0.000000\t{lat:.8f}\t{lon:.8f}\t{alt:.6f}\t1")


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                 description=__doc__)
    ap.add_argument("--direk1", type=koordinat, required=True, help="1. direk lat,lon")
    ap.add_argument("--direk2", type=koordinat, required=True, help="2. direk lat,lon")
    ap.add_argument("--kalkis", type=koordinat, required=True,
                    help="kalkis / baslangic-bitis cizgisi lat,lon")
    ap.add_argument("--tarama", type=koordinat, nargs=2, required=True,
                    metavar=("KOSE1", "KOSE2"),
                    help="tarama alaninin KARSILIKLI iki kosesi")
    ap.add_argument("--irtifa", type=float, default=cfg.TARAMA_IRTIFA_HEDEF_M)
    ap.add_argument("--direk-payi", type=float, default=25.0,
                    help="direkleri distan alirken birakilacak pay (metre)")
    ap.add_argument("--hiz", type=float, default=6.0, help="tahmini yer hizi (m/s)")
    ap.add_argument("--cikti", default="gorev2.waypoints")
    args = ap.parse_args()

    kam = geo.KameraModeli(hfov_derece=cfg.KAMERA_HFOV_DERECE)
    irtifa = args.irtifa

    if irtifa > cfg.TESPIT_MAKS_IRTIFA_M:
        print(f"!! UYARI: {irtifa} m tarama irtifasi tavanin ({cfg.TESPIT_MAKS_IRTIFA_M} m)")
        print("   ustunde. 1x1 m kirmizi hedef modelin goremeyecegi kadar kucuk kalir.")

    # --- Direk 2'yi DIŞTAN alma noktası -------------------------------------
    # Direk 1'den direk 2'ye giden yonun DEVAMINDA, direk 2'nin otesinde bir
    # nokta. Boylece direk 2 icerde kalir (sartname: distan almak zorunlu).
    (bk, bd), direkler_arasi = yon_vektoru(args.direk1, args.direk2)
    d2_disi = oteleyerek(args.direk2, bk * args.direk_payi, bd * args.direk_payi)

    # --- Tarama seritleri ---------------------------------------------------
    seritler, n_serit, genislik, aralik = tarama_seritleri(
        args.tarama[0], args.tarama[1], irtifa, kam)

    # --- Rotayi kur ---------------------------------------------------------
    rota = []            # (komut, lat, lon, alt, aciklama)
    rota.append((NAV_TAKEOFF, args.kalkis[0], args.kalkis[1], irtifa, "kalkis"))
    rota.append((NAV_WAYPOINT, args.direk1[0], args.direk1[1], irtifa,
                 "direk 1 civari"))
    rota.append((NAV_WAYPOINT, d2_disi[0], d2_disi[1], irtifa,
                 "DIREK 2 DISTAN ALINDI"))
    tespit_wp = len(rota) + 1        # +1: index 0 HOME satiridir
    for i, (lat, lon) in enumerate(seritler):
        rota.append((NAV_WAYPOINT, lat, lon, irtifa, f"tarama serit {i//2+1}"))
    rota.append((NAV_WAYPOINT, args.kalkis[0], args.kalkis[1], irtifa,
                 "bitis cizgisi"))
    rota.append((NAV_LAND, args.kalkis[0], args.kalkis[1], 0.0, "otonom inis"))

    # --- Dosyayi yaz --------------------------------------------------------
    satirlar = ["QGC WPL 110"]
    # index 0 = HOME (Mission Planner burayi kalkista kendi gunceller)
    satirlar.append(satir(0, NAV_WAYPOINT, args.kalkis[0], args.kalkis[1], 0,
                          current=1, frame=0))
    for i, (komut, lat, lon, alt, _) in enumerate(rota, start=1):
        satirlar.append(satir(i, komut, lat, lon, alt))
    Path(args.cikti).write_text("\n".join(satirlar) + "\n", encoding="utf-8")

    # --- Rapor --------------------------------------------------------------
    toplam = 0.0
    onceki = args.kalkis
    for komut, lat, lon, alt, _ in rota:
        toplam += geo.mesafe_m(onceki[0], onceki[1], lat, lon)
        onceki = (lat, lon)
    sure = toplam / max(args.hiz, 0.1)

    print("=" * 70)
    print("SAFAK UAV - GOREV 2 ROTASI URETILDI")
    print("=" * 70)
    print(f"  cikti dosyasi     : {args.cikti}")
    print(f"  tarama irtifasi   : {irtifa:.0f} m")
    print(f"  kamera serit eni  : {genislik:.1f} m  (HFOV {cfg.KAMERA_HFOV_DERECE}deg)")
    print(f"  serit araligi     : {aralik:.1f} m  (%25 ortusme)")
    print(f"  serit sayisi      : {n_serit}")
    print(f"  direkler arasi    : {direkler_arasi:.0f} m")
    print(f"  toplam rota       : {toplam:.0f} m")
    print(f"  tahmini sure      : {sure:.0f} s = {sure/60:.1f} dk "
          f"(+ birakma icin ~90 s)")
    if sure + 90 > cfg.GOREV_SURE_LIMIT_S:
        print(f"  !! UYARI: 10 dakika sinirini asiyor. Serit sayisini azaltin,")
        print(f"     tarama alanini daraltin ya da WPNAV_SPEED'i artirin.")
    print()
    print("  WAYPOINT LISTESI")
    print("  " + "-" * 66)
    print(f"  {'no':>3}  {'komut':<12} {'irtifa':>7}  aciklama")
    for i, (komut, lat, lon, alt, aciklama) in enumerate(rota, start=1):
        ad = {NAV_TAKEOFF: "TAKEOFF", NAV_LAND: "LAND"}.get(komut, "WAYPOINT")
        isaret = "  <== TESPIT ACILIR" if i == tespit_wp else ""
        print(f"  {i:>3}  {ad:<12} {alt:>6.0f}m  {aciklama}{isaret}")
    print()
    print("  >>> gorev_config.py icine yazin:")
    print(f"      TESPIT_ACILIS_WP = {tespit_wp}")
    print()
    print("  Mission Planner: Flight Plan > Load WP File > "
          f"{args.cikti} > Write WPs")
    print("  Yukledikten SONRA waypoint numaralarini ekranda dogrulayin;")
    print("  Mission Planner bazen HOME satirini farkli numaralandirir.")


if __name__ == "__main__":
    main()
