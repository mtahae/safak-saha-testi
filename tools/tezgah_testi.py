"""
ŞAFAK UAV - TEZGÂH TESTİ (Donanım Kabul Kontrolü)
==================================================
Görev yazılımını gerçek donanıma bağlamadan önce, her bileşeni TEK TEK
doğrular. S550 test dronu üzerinde, **PERVANELER SÖKÜK** hâlde çalıştırılır.

    python tools/tezgah_testi.py --baglanti /dev/serial0
    python tools/tezgah_testi.py --baglanti /dev/serial0 --test 5

!!! GÜVENLİK !!!
    - PERVANELERİ SÖKÜN. Bu script arm komutu göndermez ama servo tetikler.
    - Servo testinde bırakma mekanizmasının önünde kimse/bir şey olmasın.
    - Batarya bağlıyken araç sabit bir zemine oturmuş olsun.

Testler, en çok soruna yol açandan en aza doğru sıralanmıştır. Bir test
başarısızsa sonrakiler anlamsızdır — sırayla ilerleyin.

Her testin sonucu runs/gorev2/tezgah_raporu.txt dosyasına yazılır; uçuş
öncesi parametre listesi buradan derlenir.
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.gorev import geo, gorev_config as cfg

RAPOR = []


def yaz(*a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    RAPOR.append(s)


def sor(soru):
    try:
        return input(f"\n>>> {soru} [e/h]: ").strip().lower().startswith("e")
    except (EOFError, KeyboardInterrupt):
        return False


def baslik(no, ad):
    yaz("\n" + "=" * 66)
    yaz(f"TEST {no}: {ad}")
    yaz("=" * 66)


# ===========================================================================
def test1_baglanti(u):
    baslik(1, "MAVLINK BAGLANTISI VE TELEMETRI HIZI")
    d = u.durum()
    yaz(f"  mod            : {d.mod}")
    yaz(f"  armed          : {d.armed}")
    yaz(f"  batarya        : {d.batarya_v:.2f} V")

    # Telemetri hızını gerçekten ölç — veri akıyor demek yeterli değil,
    # geolokasyon 20 Hz duruş bekliyor.
    yaz("\n  Telemetri hizi olculuyor (5 sn)...")
    t0 = time.time()
    konum_n = durus_n = 0
    son_k = son_d = None
    while time.time() - t0 < 5.0:
        s = u.durum()
        if s.konum_yasi < 0.05 and (son_k is None or s.zaman - son_k > 0.01):
            konum_n += 1
            son_k = s.zaman
        if s.durus_yasi < 0.02 and (son_d is None or s.zaman - son_d > 0.005):
            durus_n += 1
            son_d = s.zaman
        time.sleep(0.005)

    d = u.durum()
    konum_ok = d.konum_yasi < 0.5
    durus_ok = d.durus_yasi < 0.2
    yaz(f"  konum verisi   : {'AKIYOR' if konum_ok else 'YOK/YAVAS'} "
        f"(yas {d.konum_yasi*1000:.0f} ms, hedef <500 ms)")
    yaz(f"  durus verisi   : {'AKIYOR' if durus_ok else 'YOK/YAVAS'} "
        f"(yas {d.durus_yasi*1000:.0f} ms, hedef <200 ms)")
    if not durus_ok:
        yaz("  !! Durus verisi yavas. SR*_EXTRA1 parametresini yukseltin ya da")
        yaz("     baglantinin baud hizini kontrol edin (921600 onerilir).")
    return konum_ok and durus_ok


def test2_gps(u):
    baslik(2, "GPS VE IRTIFA")
    d = u.durum()
    yaz(f"  fix tipi       : {d.gps_fix}  (3 = 3D fix, geolokasyon icin SART)")
    yaz(f"  uydu sayisi    : {d.uydu}     (>=12 iyi, <8 riskli)")
    yaz(f"  konum          : {d.lat:.7f}, {d.lon:.7f}" if d.lat else "  konum: YOK")
    yaz(f"  irtifa (AGL)   : {d.alt_agl:.2f} m  (yerdeyken ~0 olmali)")
    yaz(f"  lidar          : {f'{d.lidar_m:.2f} m' if d.lidar_m else 'YOK -> barometre kullanilacak'}")
    if d.lidar_m is None:
        yaz("  Not: Lidar yok. Barometre gun icinde metrelerce kayabilir.")
        yaz("       Her ucus oncesi yerde iken irtifanin 0 gosterdigini dogrulayin.")
    ok = d.gps_fix >= 3 and d.uydu >= 8
    if not ok:
        yaz("  !! GPS yetersiz. Acik havada, metal/bina uzaginda tekrar deneyin.")
    return ok


def test3_durus_isaret(u):
    baslik(3, "DURUS ISARET KONVANSIYONU  ***EN KRITIK TEST***")
    yaz("""
  Geolokasyonun en buyuk sistematik risk kaynagi budur. roll/pitch isareti
  ters olursa hedef HER ZAMAN yanlis tarafa hesaplanir ve yuk surekli ayni
  yone kacar. Yerde 30 saniyede dogrulanir; havada fark edilmesi cok pahali.

  Araci elinizle egip ekrandaki degerleri okuyacaksiniz.
""")
    testler = [
        ("Aracin BURNUNU yukari kaldirin (kuyruk asagi)", "pitch", +1),
        ("Aracin BURNUNU asagi indirin",                  "pitch", -1),
        ("Aracin SAG kolunu asagi indirin (saga yatirin)", "roll",  +1),
        ("Aracin SOL kolunu asagi indirin",               "roll",  -1),
    ]
    hepsi = True
    for talimat, eksen, beklenen in testler:
        input(f"\n  {talimat}, sonra ENTER'a basin...")
        d = u.durum()
        deger = math.degrees(d.pitch if eksen == "pitch" else d.roll)
        ok = (deger * beklenen > 0) and abs(deger) > 5
        hepsi &= ok
        yaz(f"  [{'OK ' if ok else 'HATA'}] {eksen} = {deger:+6.1f} deg "
            f"(beklenen isaret: {'+' if beklenen > 0 else '-'})")
        if not ok and abs(deger) <= 5:
            yaz("       (aracı daha fazla egin, en az 10 derece)")
    if not hepsi:
        yaz("\n  !! Isaretler beklenenden farkli. Ucus kontrolcusunun montaj yonu")
        yaz("     (AHRS_ORIENTATION) yanlis olabilir. DUZELTMEDEN UCMAYIN.")
    else:
        yaz("\n  Isaret konvansiyonu dogru. geo.py bu isaretlere gore yazildi.")
    return hepsi


def test4_kamera(kaynak):
    baslik(4, "KAMERA")
    import cv2
    from src.gorev.algilayici import Kamera
    try:
        kam = Kamera(kaynak, cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK,
                     cfg.KAMERA_FPS, yaz)
    except Exception as e:
        yaz(f"  [HATA] kamera acilamadi: {e}")
        if kaynak != "picam":
            yaz("       RPi Camera Module 3 icin --kaynak picam kullanin.")
        return False, None

    t0 = time.time()
    n = 0
    kare = None
    while time.time() - t0 < 3.0:
        ok, k = kam.oku()
        if ok:
            n += 1
            kare = k
    fps = n / 3.0
    if kare is None:
        yaz("  [HATA] kare okunamadi")
        return False, None
    h, w = kare.shape[:2]
    yaz(f"  cozunurluk     : {w}x{h}  (config: {cfg.KAMERA_GENISLIK}x{cfg.KAMERA_YUKSEKLIK})")
    yaz(f"  yakalama hizi  : {fps:.1f} FPS")
    boyut_ok = (w == cfg.KAMERA_GENISLIK and h == cfg.KAMERA_YUKSEKLIK)
    if not boyut_ok:
        yaz("  !! Cozunurluk config ile UYUSMUYOR. Geolokasyon kare genisligini")
        yaz("     odak uzakligi hesabinda kullanir; gorev_config'i duzeltin.")
    yol = cfg.KANIT_DIZIN / "tezgah_kamera.jpg"
    cfg.KANIT_DIZIN.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(yol), kare)
    yaz(f"  ornek kare     : {yol}")
    return boyut_ok and fps > 5, kam


def test5_tespit(kam):
    baslik(5, "HEDEF TESPITI (branda ile)")
    import cv2
    if kam is None:
        yaz("  [ATLANDI] kamera yok")
        return False
    yaz("""
  Brandayi kameranin gordugu alana YERE SERIN ve kamerayi 2-4 m yukaridan
  asagi baktirin. Model havadan, yere serili brandaya bakarak egitildi;
  elde tutulan/dik duran brandayi tanimamasi BEKLENEN davranistir.
""")
    input("  Branda hazir olunca ENTER...")
    try:
        from src.detect_ncnn import HedefDedektoruNCNN
        ded = HedefDedektoruNCNN(cfg.NCNN_MODEL_DIZIN, conf=cfg.MIN_GUVEN)
        yaz("  dedektor: NCNN")
    except Exception:
        from src.detect import HedefDedektoru
        ded = HedefDedektoru(cfg.PT_MODEL, conf=cfg.MIN_GUVEN)
        yaz("  dedektor: PyTorch")

    t0 = time.time()
    n = 0
    bulunan = {"kirmizi_hedef": 0, "mavi_hedef": 0}
    son = None
    while time.time() - t0 < 10.0:
        ok, kare = kam.oku()
        if not ok:
            continue
        s = ded.isle(kare)
        n += 1
        for k in bulunan:
            if s[k]:
                bulunan[k] += 1
        son = ded.ciz(kare, s)
    yaz(f"  10 saniyede {n} kare islendi ({n/10:.1f} FPS cikarim)")
    for k, v in bulunan.items():
        yaz(f"  {k:14s}: {v}/{n} karede bulundu ({v/max(n,1)*100:.0f}%)")
    if son is not None:
        yol = cfg.KANIT_DIZIN / "tezgah_tespit.jpg"
        cv2.imwrite(str(yol), son)
        yaz(f"  sonuc goruntusu: {yol}")
    if n / 10 < 8:
        yaz(f"  !! Cikarim hizi dusuk ({n/10:.1f} FPS). Sogutmayi ve")
        yaz("     config.CIKARIM_THREAD degerini kontrol edin.")
    return sum(bulunan.values()) > 0


def test6_servo(u):
    baslik(6, "SERVO / YUK BIRAKMA")
    yaz("""
  !!! PERVANELER SOKUK OLMALI !!!
  Her kanal tek tek tetiklenecek. Hangi mekanizmanin hareket ettigini
  gozunuzle dogrulayin -- kanal numaralari karisirsa sahada YANLIS YUK duser.

  Not: ArduPilot'ta DO_SET_SERVO'nun calismasi icin ilgili kanalin
  SERVOn_FUNCTION parametresi 0 (Disabled) olmalidir. Servo kipirdamiyorsa
  once bunu kontrol edin.
""")
    kanallar = [
        (cfg.SERVO_KANAL_MAVI_YUK, "MAVI yuk  -> 1x1 KIRMIZI hedefe (20 puan)"),
        (cfg.SERVO_KANAL_KIRMIZI_YUK, "KIRMIZI yuk -> 2x2 MAVI hedefe (10 puan)"),
    ]
    hepsi = True
    for kanal, aciklama in kanallar:
        yaz(f"\n  --- Kanal {kanal}: {aciklama} ---")
        if not sor(f"Kanal {kanal} test edilsin mi? (guvenli mi?)"):
            yaz(f"  [ATLANDI] kanal {kanal}")
            hepsi = False
            continue
        yaz(f"  kilitli konum (PWM {cfg.SERVO_KILITLI_PWM})...")
        u.servo(kanal, cfg.SERVO_KILITLI_PWM)
        time.sleep(1.5)
        yaz(f"  ACILIYOR (PWM {cfg.SERVO_ACIK_PWM})...")
        u.servo(kanal, cfg.SERVO_ACIK_PWM)
        time.sleep(cfg.SERVO_ACIK_KALMA_S)
        u.servo(kanal, cfg.SERVO_KILITLI_PWM)
        ok = sor(f"Kanal {kanal} mekanizmasi HAREKET ETTI mi ve DOGRU yuk mu?")
        hepsi &= ok
        yaz(f"  [{'OK ' if ok else 'HATA'}] kanal {kanal}")
    return hepsi


def test7_mod(u):
    baslik(7, "UCUS MODU DEGISIMI (GUIDED)")
    yaz("""
  Gorev yazilimi, yuk birakirken AUTO'dan GUIDED'a gecer. Bu gecisin
  calistigini yerde dogruluyoruz. Arac DISARM, pervaneler sokuk.
""")
    baslangic = u.durum().mod
    yaz(f"  baslangic modu : {baslangic}")
    ok = u.mod_ayarla("GUIDED")
    yaz(f"  [{'OK ' if ok else 'HATA'}] GUIDED'a gecis")
    if not ok:
        yaz("  !! GUIDED reddedildi. Yaygin sebep: 3D GPS fix yok ya da EKF")
        yaz("     henuz oturmadi. Acik havada, GPS fix aldiktan sonra deneyin.")
    time.sleep(1.0)
    geri = u.mod_ayarla("LOITER") or u.mod_ayarla("STABILIZE")
    yaz(f"  [{'OK ' if geri else 'HATA'}] guvenli moda donus")
    return ok


def test8_yayin(kam):
    baslik(8, "YER ISTASYONU CANLI YAYINI")
    yaz(f"""
  Yer istasyonu bilgisayarinda SU KOMUTU calistirin:
      python tools/yer_istasyonu_izle.py --port {cfg.YAYIN_PORT}

  Hedef IP (gorev_config.YAYIN_HEDEF_IP) = {cfg.YAYIN_HEDEF_IP}
  Yer istasyonunun IP'si bu degilse gorev_config.py'yi duzeltin.
""")
    if kam is None:
        yaz("  [ATLANDI] kamera yok")
        return False
    if not sor("Yer istasyonunda izleyici calisiyor mu?"):
        yaz("  [ATLANDI]")
        return False
    from src.gorev.yayin import Yayin
    y = Yayin(cfg.YAYIN_HEDEF_IP, cfg.YAYIN_PORT, yaz)
    son = {"k": None}

    def al():
        ok, k = kam.oku()
        son["k"] = k if ok else son["k"]
        return son["k"]

    y.basla(al)
    yaz("  15 saniye yayin yapiliyor...")
    time.sleep(15)
    y.durdur()
    yaz(f"  gonderilen kare: {y.gonderilen}")
    ok = sor("Yer istasyonunda GORUNTU gorundu mu?")
    yaz(f"  [{'OK ' if ok else 'HATA'}] canli yayin")
    if not ok:
        yaz("  !! Guvenlik duvarini (Windows Defender) ve ayni agda olup")
        yaz("     olmadiginizi kontrol edin. UDP 5600 acik olmali.")
    return ok


# ===========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baglanti", default=cfg.BAGLANTI)
    ap.add_argument("--baud", type=int, default=cfg.BAUD)
    ap.add_argument("--kaynak", default="picam")
    ap.add_argument("--test", type=int, default=None,
                    help="sadece bu numarali testi calistir")
    args = ap.parse_args()

    yaz("=" * 66)
    yaz("SAFAK UAV - TEZGAH TESTI")
    yaz(f"zaman: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    yaz(f"baglanti: {args.baglanti} @ {args.baud}   kamera: {args.kaynak}")
    yaz("!!! PERVANELER SOKUK OLMALI !!!")
    yaz("=" * 66)

    from src.gorev.ucus import Ucus
    u = None
    kam = None
    sonuclar = {}
    try:
        kamera_gerekli = args.test in (None, 4, 5, 8)
        ucus_gerekli = args.test in (None, 1, 2, 3, 6, 7)

        if ucus_gerekli:
            u = Ucus(args.baglanti, args.baud, cfg.KENDI_SYSTEM_ID,
                     cfg.KENDI_COMPONENT_ID, yaz)
            u.hazir_bekle(zaman_asimi=30)

        sirali = [
            (1, "MAVLink baglantisi", lambda: test1_baglanti(u), ucus_gerekli),
            (2, "GPS", lambda: test2_gps(u), ucus_gerekli),
            (3, "Durus isaretleri", lambda: test3_durus_isaret(u), ucus_gerekli),
            (4, "Kamera", None, kamera_gerekli),
            (5, "Tespit", lambda: test5_tespit(kam), kamera_gerekli),
            (6, "Servo", lambda: test6_servo(u), ucus_gerekli),
            (7, "Mod degisimi", lambda: test7_mod(u), ucus_gerekli),
            (8, "Canli yayin", lambda: test8_yayin(kam), kamera_gerekli),
        ]
        for no, ad, fn, uygun in sirali:
            if args.test is not None and args.test != no:
                continue
            if not uygun:
                continue
            if no == 4:
                ok, kam = test4_kamera(args.kaynak)
                sonuclar[no] = (ad, ok)
                continue
            if no in (5, 8) and kam is None:
                # Tek test calistiriliyorsa (--test 5) kamera henuz acilmamistir
                _, kam = test4_kamera(args.kaynak)
            sonuclar[no] = (ad, fn())
    except KeyboardInterrupt:
        yaz("\n[iptal] kullanici durdurdu")
    except Exception as e:
        import traceback
        yaz(f"\n[HATA] {e}\n{traceback.format_exc()}")
    finally:
        if kam:
            kam.kapat()
        if u:
            u.kapat()

    yaz("\n" + "=" * 66)
    yaz("OZET")
    yaz("=" * 66)
    for no, (ad, ok) in sorted(sonuclar.items()):
        yaz(f"  {no}. {ad:24s} {'GECTI' if ok else 'BASARISIZ / EKSIK'}")

    cfg.KANIT_DIZIN.mkdir(parents=True, exist_ok=True)
    yol = cfg.KANIT_DIZIN / "tezgah_raporu.txt"
    with open(yol, "a", encoding="utf-8") as f:
        f.write("\n".join(RAPOR) + "\n\n")
    print(f"\nRapor: {yol}")


if __name__ == "__main__":
    main()
