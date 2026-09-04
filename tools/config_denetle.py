"""
ŞAFAK UAV - Yapılandırma Denetleyicisi
=======================================
`gorev_config.py` sahada elle düzenlenecek. Tutarsız bir değer kombinasyonu
çökme üretmez — SESSİZCE yanlış davranış üretir ve bunu ancak uçuş sırasında,
puan kaybederek fark edersiniz.

Bu araç, uçuştan önce o kombinasyonları kontrol eder.

    python tools/config_denetle.py

Çıkış kodu: 0 = temiz, 1 = HATA var (uçmayın), 2 = sadece uyarı var.

Her uçuştan önce çalıştırın. `tools/tezgah_testi.py` de bunu çağırmadan önce
elle koşturmak iyi bir alışkanlıktır.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.gorev import geo, gorev_config as cfg

HATALAR = []
UYARILAR = []
BILGI = []


def hata(mesaj, neden):
    HATALAR.append((mesaj, neden))


def uyari(mesaj, neden):
    UYARILAR.append((mesaj, neden))


def denetle():
    kam = geo.KameraModeli(hfov_derece=cfg.KAMERA_HFOV_DERECE,
                           montaj_yaw_derece=cfg.KAMERA_MONTAJ_YAW_DERECE)
    G = cfg.KAMERA_GENISLIK

    # --- Servo kanalları -----------------------------------------------------
    if cfg.SERVO_KANAL_KIRMIZI_YUK == cfg.SERVO_KANAL_MAVI_YUK:
        hata(f"Iki yuk ayni servo kanalinda ({cfg.SERVO_KANAL_KIRMIZI_YUK})",
             "Birinci birakmada IKI yuk birden duser; ikinci hedef icin yuk kalmaz.")
    for ad in ("SERVO_KANAL_KIRMIZI_YUK", "SERVO_KANAL_MAVI_YUK"):
        k = getattr(cfg, ad)
        if not (1 <= k <= 16):
            hata(f"{ad}={k} gecersiz", "Pixhawk servo kanallari 1-16 arasindadir.")
    if cfg.SERVO_ACIK_PWM == cfg.SERVO_KILITLI_PWM:
        hata("SERVO_ACIK_PWM ile SERVO_KILITLI_PWM ayni",
             "Servo hic hareket etmez, yuk birakilamaz.")
    for ad in ("SERVO_ACIK_PWM", "SERVO_KILITLI_PWM"):
        v = getattr(cfg, ad)
        if not (800 <= v <= 2200):
            uyari(f"{ad}={v} olagandisi",
                  "Tipik PWM araligi 1000-2000 us. Servoyu zorlayabilir.")

    # --- Yük eşlemesi (ÇAPRAZ olmalı — şartname) ------------------------------
    beklenen = {
        "mavi_hedef": cfg.SERVO_KANAL_KIRMIZI_YUK,      # 2x2 mavi <- kirmizi yuk
        "kirmizi_hedef": cfg.SERVO_KANAL_MAVI_YUK,      # 1x1 kirmizi <- mavi yuk
    }
    for sinif, kanal in beklenen.items():
        if sinif not in cfg.YUK_ESLEME:
            hata(f"YUK_ESLEME'de {sinif} yok", "Bu hedefe yuk birakilamaz.")
        elif cfg.YUK_ESLEME[sinif][1] != kanal:
            hata(f"YUK_ESLEME[{sinif}] kanal {cfg.YUK_ESLEME[sinif][1]}, "
                 f"beklenen {kanal}",
                 "Sartname CAPRAZ eslesme istiyor: 2x2 MAVI hedefe disi KIRMIZI "
                 "yuk, 1x1 KIRMIZI hedefe disi MAVI yuk. Yanlis renk = 0 puan.")

    # --- İrtifa zinciri ------------------------------------------------------
    if cfg.MIN_IRTIFA_M >= cfg.BIRAKMA_IRTIFA_M:
        hata(f"MIN_IRTIFA_M ({cfg.MIN_IRTIFA_M}) >= BIRAKMA_IRTIFA_M "
             f"({cfg.BIRAKMA_IRTIFA_M})",
             "Hedefin tam ustunde, birakma irtifasinda alinan EN ISABETLI "
             "olcumler elenir ve birakma oncesi son duzeltme calismaz.")
    elif cfg.BIRAKMA_IRTIFA_M - cfg.MIN_IRTIFA_M < 1.0:
        uyari(f"MIN_IRTIFA_M ile BIRAKMA_IRTIFA_M cok yakin "
              f"({cfg.MIN_IRTIFA_M} / {cfg.BIRAKMA_IRTIFA_M})",
              "Irtifa gurultusu olcumlerin bir kismini eleyebilir. "
              "Aralarinda en az 1-2 m birakin.")
    if cfg.BIRAKMA_IRTIFA_M > cfg.YAKLASMA_IRTIFA_M:
        hata("BIRAKMA_IRTIFA_M > YAKLASMA_IRTIFA_M",
             "Yaklasma alcalma degil tirmanma olur; mantik ters.")
    if cfg.BIRAKMA_IRTIFA_M < 2.0:
        uyari(f"BIRAKMA_IRTIFA_M={cfg.BIRAKMA_IRTIFA_M} m cok alcak",
              "Yer etkisi ve iniş takimi carpma riski. 4-6 m onerilir.")
    if cfg.TARAMA_IRTIFA_HEDEF_M > cfg.TESPIT_MAKS_IRTIFA_M:
        hata(f"TARAMA_IRTIFA_HEDEF_M ({cfg.TARAMA_IRTIFA_HEDEF_M}) > "
             f"TESPIT_MAKS_IRTIFA_M ({cfg.TESPIT_MAKS_IRTIFA_M})",
             "Model bu irtifada 1x1 m hedefi goremez.")

    # --- Tolerans zinciri ----------------------------------------------------
    if cfg.SON_KONUM_TOLERANS_M > cfg.KONUM_TOLERANS_M:
        uyari("SON_KONUM_TOLERANS_M > KONUM_TOLERANS_M",
              "Son yaklasma, kaba yaklasmadan daha GEVSEK. Hassasiyet kazanci yok.")
    if cfg.SON_KONUM_TOLERANS_M < 0.2:
        uyari(f"SON_KONUM_TOLERANS_M={cfg.SON_KONUM_TOLERANS_M} m cok siki",
              "ArduPilot GUIDED bu banda oturamayabilir; her birakmada "
              "SON_YAKLASMA_ZAMAN_ASIMI_S kadar bosuna beklenir.")
    if cfg.MAKS_KUME_DAGILIM_M > cfg.KUME_YARICAP_M:
        uyari(f"MAKS_KUME_DAGILIM_M ({cfg.MAKS_KUME_DAGILIM_M}) > "
              f"KUME_YARICAP_M ({cfg.KUME_YARICAP_M})",
              "Dagilim kontrolu hicbir kumeyi eleyemez; kalite kapisi etkisiz.")

    # --- Süre bütçesi --------------------------------------------------------
    if cfg.BIRAKMAYI_BIRAK_S >= cfg.GOREV_SURE_LIMIT_S:
        hata("BIRAKMAYI_BIRAK_S >= GOREV_SURE_LIMIT_S",
             "Birakma denemeleri 10 dk sinirina kadar surer; bitis cizgisi ve "
             "inis icin sure kalmaz, ucus BASARISIZ sayilir.")
    kalan = cfg.GOREV_SURE_LIMIT_S - cfg.BIRAKMAYI_BIRAK_S
    if kalan < 90:
        uyari(f"Birakma sonrasi sadece {kalan:.0f} s kaliyor",
              "Bitis cizgisini gecip inmek icin dar. 2 dakika birakin.")
    tek_birakma = (cfg.VARIS_ZAMAN_ASIMI_S * 2 + cfg.SON_YAKLASMA_ZAMAN_ASIMI_S
                   + cfg.DURULMA_SANIYE + cfg.SERVO_ACIK_KALMA_S)
    if tek_birakma * 2 > cfg.BIRAKMAYI_BIRAK_S:
        uyari(f"En kotu halde iki birakma {tek_birakma*2:.0f} s surebilir, "
              f"butce {cfg.BIRAKMAYI_BIRAK_S} s",
              "Zaman asimlarini kisaltin ya da butceyi buyutun.")

    # --- Kamera / geolokasyon ------------------------------------------------
    # Bilinen KATALOG degerleri. Bunlardan biri girilmisse deger olculmemis
    # demektir -- IMX219'da kirpilmis sensor modu katalog degerini gecersiz
    # kilar (1280x720 -> 50.4 deg), bu yuzden olcum sart.
    KATALOG = {66.0: "IMX708 standart", 102.0: "IMX708 genis",
               62.2: "IMX219 tam FOV", 63.6: "IMX219 (satici f=2.96)"}
    if cfg.KAMERA_HFOV_DERECE in KATALOG:
        uyari(f"KAMERA_HFOV_DERECE={cfg.KAMERA_HFOV_DERECE} katalog degeri "
              f"({KATALOG[cfg.KAMERA_HFOV_DERECE]})",
              "Henuz OLCULMEMIS. IMX219'da bazi cozunurlukler sensoru KIRPAR "
              "(1920x1080 -> 38.9 deg, 1280x720 -> 50.4 deg, 640x480 -> 26.5 deg); "
              "kirpma varsa katalog degeri gecersizdir ve TUM mesafeler ayni "
              "oranda kayar. python tools/kamera_kalibrasyon.py --hfov ile olcun.")
    else:
        BILGI.append(f"HFOV {cfg.KAMERA_HFOV_DERECE} deg - katalog degeri degil, "
                     f"olculmus gorunuyor")

    # IMX219'da kirpilmis oldugu BILINEN cozunurlukler
    KIRPIK_MOD = {(1920, 1080): 38.9, (1280, 720): 50.4, (640, 480): 26.5}
    mod = (cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK)
    if mod in KIRPIK_MOD:
        uyari(f"Cozunurluk {mod[0]}x{mod[1]} IMX219'da KIRPILMIS bir moddur",
              f"Eski libcamera yolunda bu modun gercek HFOV'u ~{KIRPIK_MOD[mod]} deg "
              f"olur ({cfg.KAMERA_HFOV_DERECE} deg degil) ve mesafeler "
              f"%{(math.tan(math.radians(cfg.KAMERA_HFOV_DERECE/2))/math.tan(math.radians(KIRPIK_MOD[mod]/2))-1)*100:.0f} "
              f"siser. picamera2 her zaman kirpmaz -- ama kirparsa hata VERMEZ. "
              f"Tam FOV garantisi icin 1640x1232 veya 1640x922 kullanin, ya da "
              f"kamera_kalibrasyon.py --hfov ile olcup gercek degeri girin.")
    if cfg.KAMERA_MONTAJ_YAW_DERECE not in (0, 90, 180, 270):
        uyari(f"KAMERA_MONTAJ_YAW_DERECE={cfg.KAMERA_MONTAJ_YAW_DERECE} "
              f"90'in kati degil",
              "Kamera egik monte edildiyse dogru; degilse olcum hatasi olabilir.")
    if cfg.MAKS_EGIM_DERECE > 40:
        uyari(f"MAKS_EGIM_DERECE={cfg.MAKS_EGIM_DERECE} yuksek",
              "Sert virajda alinan olcumler havuzu bozar.")

    # --- Türetilmiş büyüklükler (bilgi) --------------------------------------
    irt = cfg.TARAMA_IRTIFA_HEDEF_M
    f = kam.odak_piksel(G)
    serit = 2.0 * irt * (G / 2.0) / f
    gsd = geo.yer_ornekleme_m(irt, G, kam)
    hedef_px_1x1 = 1.0 / gsd
    hedef_px_2x2 = 2.0 / gsd
    BILGI.append(f"Tarama {irt:.0f} m: serit eni {serit:.1f} m, "
                 f"GSD {gsd*100:.1f} cm/px")
    BILGI.append(f"  1x1 m hedef {hedef_px_1x1:.0f} px, "
                 f"2x2 m hedef {hedef_px_2x2:.0f} px")
    if hedef_px_1x1 < 35:
        hata(f"1x1 m hedef tarama irtifasinda sadece {hedef_px_1x1:.0f} piksel",
             "Model ~35 pikselin altini goremiyor (olculdu). "
             "Tarama irtifasini dusurun.")
    elif hedef_px_1x1 < 45:
        uyari(f"1x1 m hedef {hedef_px_1x1:.0f} piksel - sinira yakin",
              "20 m'de ~49 px ile %100 recall olculmustu. Marj birakin.")
    BILGI.append(f"Durus 1 derece hata -> {irt*math.tan(math.radians(1))*100:.0f} cm "
                 f"yer hatasi (tarama irtifasinda)")

    if cfg.BIRAKMA_OFSET_KUZEY_M == 0 and cfg.BIRAKMA_OFSET_DOGU_M == 0:
        BILGI.append("BIRAKMA_OFSET_* = 0 (henuz gercek atis yapilmadi). "
                     "Ilk atistan sonra tools/birakma_ofset.py ile doldurun.")

    # --- Yollar --------------------------------------------------------------
    if not cfg.NCNN_MODEL_DIZIN.exists() and not cfg.PT_MODEL.exists():
        hata("Ne NCNN ne PyTorch modeli bulunamadi",
             f"{cfg.NCNN_MODEL_DIZIN} ve {cfg.PT_MODEL} yok.")
    elif not cfg.NCNN_MODEL_DIZIN.exists():
        uyari("NCNN modeli yok, PyTorch kullanilacak",
              "RPi5'te PyTorch cok yavas. tools/export_model.py --format ncnn")


def main():
    print("=" * 70)
    print("SAFAK UAV - YAPILANDIRMA DENETIMI")
    print("=" * 70)
    denetle()

    if BILGI:
        print("\nBILGI")
        for b in BILGI:
            print(f"  - {b}")

    if UYARILAR:
        print(f"\nUYARI ({len(UYARILAR)})")
        for m, n in UYARILAR:
            print(f"  ! {m}")
            print(f"      {n}")

    if HATALAR:
        print(f"\nHATA ({len(HATALAR)}) -- BU HALIYLE UCMAYIN")
        for m, n in HATALAR:
            print(f"  X {m}")
            print(f"      {n}")

    print("\n" + "=" * 70)
    if HATALAR:
        print(f"SONUC: {len(HATALAR)} HATA, {len(UYARILAR)} uyari. UCMAYIN.")
        return 1
    if UYARILAR:
        print(f"SONUC: hata yok, {len(UYARILAR)} uyari. Uyarilari okuyun.")
        return 2
    print("SONUC: yapilandirma temiz.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
