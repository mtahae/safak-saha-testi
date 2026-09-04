"""
ŞAFAK UAV - GÖREV 2 UÇTAN UCA PROVA (donanımsız)
=================================================
Tüm görev yazılımını sahte bir Pixhawk ve sahte bir kameraya karşı koşturur.
Amaç, sahaya çıkmadan şu soruların cevabını almak:

    - Durum makinesi doğru sırayla mı ilerliyor?
    - Tespit, doğru waypoint'te mi açılıyor (şartname: 2. direkten SONRA)?
    - AUTO doğru anda kesilip GUIDED'a geçiliyor mu?
    - Araç gerçekten hedefin üstüne gidiyor mu?
    - Servo DOĞRU kanaldan mı tetikleniyor? (çapraz renk eşlemesi!)
    - Bırakma sonrası AUTO'ya, doğru waypoint'ten dönülüyor mu?
    - Bulunan hedef konumu gerçeğinden kaç metre sapıyor?

Sahte kamera, hedefleri GERÇEK projeksiyon matematiğiyle (geo.gps_piksel)
kareye çizer — yani araç uçtukça hedefler kadrajda doğru yerde, doğru boyutta
ve doğru perspektifte görünür. Böylece geolokasyon zinciri kendi kendini
doğrulamaz; ileri yön (çizim) ve geri yön (çözüm) birbirinden bağımsızdır.

    python tools/gorev_prova.py                 # hizli mantik testi
    python tools/gorev_prova.py --dedektor gercek   # gercek YOLO modeliyle
    python tools/gorev_prova.py --goster        # sahte kamera goruntusunu izle
"""
import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.gorev import geo, gorev_config as cfg
from tools.sahte_pixhawk import SahtePixhawk, _oteleme

EV_LAT, EV_LON = 39.9334, 32.8597

# (kuzey_m, dogu_m, irtifa_m, tip) — waypoint sırası MISSION_CURRENT ile aynı
ROTA = [
    (0,    0,   0,  "HOME"),
    (0,    0,   20, "TAKEOFF"),
    (60,   0,   20, "WP"),          # direk 1'e dogru
    (120, -30,  20, "WP"),          # direk 2'yi distan al
    (120, -60,  20, "WP"),          # <-- TESPIT_ACILIS_WP: tarama alanina giris
    (60,  -60,  20, "WP"),          # tarama gecisi
    (0,   -20,  20, "WP"),          # bitis cizgisi
    (0,    0,   0,  "LAND"),
]

# Sahadaki gerçek hedefler: (sınıf, kuzey_m, doğu_m, kenar_m)
HEDEFLER = [
    ("kirmizi_hedef", 100.0, -57.0, 1.0),   # 1x1 m kirmizi  -> MAVI yuk (20 puan)
    ("mavi_hedef",     75.0, -63.0, 2.0),   # 2x2 m mavi     -> KIRMIZI yuk (10 puan)
]
RENK_BGR = {"kirmizi_hedef": (40, 40, 200), "mavi_hedef": (200, 120, 40)}


@dataclass
class SahteHedef:
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


class SahteKamera:
    """Simülatörün gerçek durumundan sahne üretir.

    Hedeflerin dört köşesi geo.gps_piksel ile ayrı ayrı yansıtılır; böylece
    eğik bakışta hedef gerçekten perspektif bozulmasına uğrar (kare, yamuk
    olarak görünür) — tıpkı sahadaki gibi.
    """

    def __init__(self, sim, genislik, yukseklik, kam_modeli, log=print):
        self.sim = sim
        self.G, self.Y = genislik, yukseklik
        self.kam = kam_modeli
        self.log = log
        self.hedef_gps = []
        for sinif, k, d, kenar in HEDEFLER:
            lat, lon = _oteleme(EV_LAT, EV_LON, k, d)
            self.hedef_gps.append((sinif, lat, lon, kenar))
        self._zemin = self._zemin_uret()

    def _zemin_uret(self):
        """Çim benzeri dokulu zemin — düz renk, dedektörü gerçekçi zorlamaz."""
        rng = np.random.default_rng(7)
        z = np.zeros((self.Y, self.G, 3), np.uint8)
        z[:, :] = (60, 110, 70)
        gurultu = rng.normal(0, 14, (self.Y, self.G, 1)).astype(np.int16)
        z = np.clip(z.astype(np.int16) + gurultu, 0, 255).astype(np.uint8)
        return cv2.GaussianBlur(z, (5, 5), 0)

    def oku(self):
        with self.sim._kilit:
            lat, lon, alt = self.sim.lat, self.sim.lon, self.sim.alt
            roll, pitch, yaw = self.sim.roll, self.sim.pitch, self.sim.yaw
        kare = self._zemin.copy()
        if alt < 1.0:
            return True, kare

        for sinif, h_lat, h_lon, kenar in self.hedef_gps:
            kose_piksel = []
            for dk, dd in ((-.5, -.5), (-.5, .5), (.5, .5), (.5, -.5)):
                k_lat, k_lon = _oteleme(h_lat, h_lon, dk * kenar, dd * kenar)
                p = geo.gps_piksel(k_lat, k_lon, self.G, self.Y, lat, lon, alt,
                                   roll, pitch, yaw, self.kam)
                if p is None:
                    kose_piksel = []
                    break
                kose_piksel.append([p[0], p[1]])
            if len(kose_piksel) != 4:
                continue
            pts = np.array(kose_piksel, np.int32)
            if pts[:, 0].max() < 0 or pts[:, 0].min() > self.G:
                continue
            if pts[:, 1].max() < 0 or pts[:, 1].min() > self.Y:
                continue
            cv2.fillPoly(kare, [pts], RENK_BGR[sinif])
        return True, kare

    def kapat(self):
        pass


class SahteDedektor:
    """HSV ile renkli kareleri bulur. Gerçek YOLO'nun yerine geçer.

    Bugünün amacı MODELİ değil GÖREV MANTIĞINI test etmek: doğru waypoint'te
    tespit açılıyor mu, doğru servo tetikleniyor mu, AUTO'ya dönülüyor mu.
    Model zaten test edilmiş (mAP50 0.994). --dedektor gercek ile YOLO da
    devreye alınabilir.
    """

    def __init__(self, log=print):
        from src import color_verify, config
        self.cv_ = color_verify
        self.config = config
        self.log = log

    def isle(self, bgr):
        H, W = bgr.shape[:2]
        mx, my = W / 2.0, H / 2.0
        tum = []
        for sid, isim in enumerate(self.config.SINIF_ISIMLERI):
            maske = self.cv_.renk_maskesi(bgr, sid)
            konturlar, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            for c in konturlar:
                alan = cv2.contourArea(c)
                if alan < 60:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w / 2.0, y + h / 2.0
                tum.append(SahteHedef(
                    sinif_id=sid, sinif_isim=isim, guven=0.92,
                    kutu=(x, y, x + w, y + h), cx=cx, cy=cy,
                    dx=(cx - mx) / mx, dy=(cy - my) / my,
                    alan=int(alan),
                    renk_orani=self.cv_.renk_orani(bgr, (x, y, x + w, y + h), sid)))
        en_iyi = {0: None, 1: None}
        for h in tum:
            if en_iyi[h.sinif_id] is None or h.alan > en_iyi[h.sinif_id].alan:
                en_iyi[h.sinif_id] = h
        return {"kirmizi_hedef": en_iyi[0], "mavi_hedef": en_iyi[1],
                "tum": tum, "kare_boyut": (W, H)}

    def ciz(self, bgr, sonuc):
        out = bgr.copy()
        for h in sonuc["tum"]:
            x1, y1, x2, y2 = [int(v) for v in h.kutu]
            renk = RENK_BGR[h.sinif_isim]
            cv2.rectangle(out, (x1, y1), (x2, y2), renk, 2)
            cv2.putText(out, f"{h.sinif_isim} {h.guven:.2f}", (x1, max(12, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 2)
        h, w = out.shape[:2]
        cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dedektor", choices=["sahte", "gercek"], default="sahte")
    ap.add_argument("--goster", action="store_true", help="sahte kamera goruntusunu izle")
    ap.add_argument("--port", type=int, default=5763)
    ap.add_argument("--gurultu", action="store_true",
                    help="telemetriye gercekci GPS/durus gurultusu bindir")
    ap.add_argument("--sadece-mavi", action="store_true",
                    help="saha testi bayragini prova ile dogrula")
    ap.add_argument("--temsili-servo", action="store_true")
    ap.add_argument("--ui", action="store_true", help="Flask arayuzunu de baslat")
    ap.add_argument("--senaryo", choices=["normal", "devralma"], default="normal",
                    help="devralma: birakma ortasinda pilot kumandadan modu degistirir")
    args = ap.parse_args()

    from src.gorev.gorev2 import Gorev2, Kayitci

    log = Kayitci(cfg.KANIT_DIZIN)
    log("=" * 66)
    log("SAFAK UAV - GOREV 2 UCTAN UCA PROVA")
    log("=" * 66)

    sim = SahtePixhawk(f"tcpin:127.0.0.1:{args.port}", EV_LAT, EV_LON, ROTA, log,
                       gurultu=args.gurultu)
    if args.gurultu:
        log("[prova] TELEMETRI GURULTUSU ACIK (GPS bias+gurultu, durus, irtifa)")
    sim.start()
    time.sleep(0.5)

    kam_modeli = geo.KameraModeli(hfov_derece=cfg.KAMERA_HFOV_DERECE,
                                  montaj_yaw_derece=cfg.KAMERA_MONTAJ_YAW_DERECE)
    kamera = SahteKamera(sim, cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK, kam_modeli, log)

    class Args:
        sadece_mavi = args.sadece_mavi
        temsili_servo = args.temsili_servo
        ui = args.ui
        ui_port = 5000
        baglanti = f"tcp:127.0.0.1:{args.port}"
        baud = 115200
        kaynak = "sahte"
        motor = "oto"
        prova = False          # GERÇEK komutlar gönderilsin — test edilen bu
        yayin_yok = True

    g = Gorev2(Args(), log, kamera=kamera)
    if args.dedektor == "sahte":
        g.ded = SahteDedektor(log)
        g.algi.ded = g.ded
        log("[prova] dedektor: SAHTE (HSV) — gorev mantigi test ediliyor")
    else:
        log("[prova] dedektor: GERCEK YOLO")

    # Görevi başlat: kumandadan arm + AUTO yapılmış gibi
    def baslatici():
        time.sleep(2.0)
        with sim._kilit:
            sim.armed = True
            sim.mod = "AUTO"
            sim.wp_no = 1
        log("[prova] sanal kumanda: ARM + AUTO")

    import threading
    threading.Thread(target=baslatici, daemon=True).start()

    if args.senaryo == "devralma":
        def devralma_tetikle():
            # GUIDED'a gecilmesini bekle (yazilim birakmaya basladi), sonra
            # pilot kumandadan LOITER'a alsin -- gercek bir devralma boyle olur.
            while True:
                with sim._kilit:
                    guided = (sim.mod == "GUIDED")
                if guided:
                    break
                time.sleep(0.05)
            time.sleep(4.0)
            with sim._kilit:
                sim.mod = "LOITER"
                sim.guided_hedef = None
                sim.pilot_devraldi_t = time.time()
            log("[prova] *** SANAL PILOT DEVRALDI: mod -> LOITER ***")
            # Pilot elle indirip disarm etsin. Bu olmadan gorev yazilimi
            # (dogru sekilde) aracin disarm olmasini beklemeye devam eder ve
            # test 12 dakika surer.
            time.sleep(10.0)
            with sim._kilit:
                sim.armed = False
            log("[prova] sanal pilot indirdi ve disarm etti")
        threading.Thread(target=devralma_tetikle, daemon=True).start()
        log("[prova] SENARYO: devralma (birakma ortasinda pilot mudahalesi)")

    izleyici = None
    if args.goster:
        def izle():
            while True:
                k = g.algi.kare_al()
                if k is not None:
                    cv2.imshow("SAHTE KAMERA", k)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
        izleyici = threading.Thread(target=izle, daemon=True)

    try:
        if izleyici:
            izleyici.start()
        g.calistir()
    finally:
        g.kapat()
        sim.durdur()
        cv2.destroyAllWindows()

    # ---------------- DEĞERLENDİRME ----------------
    log("\n" + "=" * 66)
    log("PROVA SONUCU")
    log("=" * 66)
    gecti = True
    # Devralma senaryosunda gorevin YARIDA KALMASI beklenen sonuctur: pilot
    # kontrolu aldigi icin ikinci yuk birakilmaz ve gorev AUTO'da bitmez.
    # Bu yuzden 1-3. bolumler orada gecme sarti degil, sadece bilgidir;
    # gecme sarti 5. bolumdur (yazilim komut gondermeyi birakti mi).
    normal = (args.senaryo == "normal")
    if not normal:
        log("(devralma senaryosu: 1-3. bolumler bilgi amaclidir, "
            "gecme sarti 5. bolumdur)")

    # SAHA TESTI BAYRAKLARI degerlendirmeyi de degistirir. Bunlar verildiginde
    # "servo acilmadi" ya da "kirmizi hedefe gidilmedi" BEKLENEN sonuctur;
    # hata olarak raporlamak yanlis alarm uretir ve gercek bir hatayi
    # gormemize engel olur.
    servo_beklenir = not args.temsili_servo
    kirmizi_beklenir = not args.sadece_mavi
    if args.temsili_servo:
        log("(temsili servo: 2. bolum bilgi amaclidir -- servo zaten "
            "tetiklenmiyor)")
    if args.sadece_mavi:
        log("(sadece mavi: kirmizi hedef icin birakma beklenmiyor)")

    log("\n1) HEDEF KONUM DOGRULUGU")
    for sinif, k, d, kenar in HEDEFLER:
        g_lat, g_lon = _oteleme(EV_LAT, EV_LON, k, d)
        kumeler = g.havuz.dogrulanmis(sinif)
        if not kumeler:
            log(f"   [{'HATA' if normal else 'bilgi'}] {sinif}: HIC BULUNAMADI")
            gecti &= (not normal)
            continue
        b_lat, b_lon = kumeler[0].konum
        hata = geo.mesafe_m(g_lat, g_lon, b_lat, b_lon)
        ok = hata < 2.0
        if normal:
            gecti &= ok
        log(f"   [{'OK ' if ok else 'HATA'}] {sinif}: hata={hata:.2f} m "
            f"(n={kumeler[0].sayi} dagilim={kumeler[0].dagilim_m:.2f}m)")

    log("\n2) SERVO TETIKLEME (capraz renk eslemesi)")
    beklenen = {cfg.SERVO_KANAL_MAVI_YUK: ("kirmizi_hedef", "1x1 KIRMIZI hedefe MAVI yuk"),
                cfg.SERVO_KANAL_KIRMIZI_YUK: ("mavi_hedef", "2x2 MAVI hedefe KIRMIZI yuk")}
    acilanlar = [r for r in sim.servo_kayit if r["pwm"] == cfg.SERVO_ACIK_PWM]
    for kanal, (sinif, aciklama) in beklenen.items():
        # Bu kanal icin servo BEKLENIYOR mu? --temsili-servo hicbirini,
        # --sadece-mavi ise kirmiziyi beklemez.
        bu_beklenir = (normal and servo_beklenir
                       and (kirmizi_beklenir or sinif == "mavi_hedef"))
        var = any(r["kanal"] == kanal for r in acilanlar)
        if bu_beklenir:
            gecti &= var
        log(f"   [{'OK ' if var else ('HATA' if bu_beklenir else 'bilgi')}] "
            f"kanal {kanal}: {aciklama}")
    hedef_sayi = 2 if kirmizi_beklenir else 1
    if not servo_beklenir:
        log(f"   [bilgi] {len(acilanlar)} servo acildi -- temsili servo "
            f"modunda 0 beklenir")
    elif len(acilanlar) != hedef_sayi:
        log(f"   [{'HATA' if normal else 'bilgi'}] {len(acilanlar)} servo "
            f"acildi, bu kosumda {hedef_sayi} beklenir")
        if normal:
            gecti = False

    log("\n2b) FIZIKSEL ISABET — puanin geldigi sayi")
    log("     servo tetiklendigi andaki GERCEK konum ile hedef merkezi arasi.")
    log("     Gorev yaziliminin kendi (gurultulu) tahmini DEGIL; GPS biasi")
    log("     burada kendini goturur, gercek isabet bu satirlarda gorunur.")
    for kanal, (sinif, _) in beklenen.items():
        kayit = next((r for r in acilanlar if r["kanal"] == kanal), None)
        if kayit is None:
            continue
        h = next(h for h in HEDEFLER if h[0] == sinif)
        g_lat, g_lon = _oteleme(EV_LAT, EV_LON, h[1], h[2])
        isabet = geo.mesafe_m(kayit["lat"], kayit["lon"], g_lat, g_lon)
        ok = isabet < 10.0
        if normal:
            gecti &= ok
        log(f"   [{'OK ' if ok else 'HATA'}] {sinif:14s} isabet={isabet:5.2f} m "
            f"(birakma irtifasi {kayit['alt']:.1f} m)")

    log("\n3) GOREVE DONUS")
    son_mod = sim.mod
    ok = son_mod in ("AUTO", "LAND", "STABILIZE")
    if normal:
        gecti &= ok
    log(f"   [{'OK ' if ok else ('HATA' if normal else 'bilgi')}] "
        f"son mod: {son_mod}, son wp: {sim.wp_no}/{len(ROTA)-1}, "
        f"armed={sim.armed}")

    log("\n4) HAKEM KANITI (STATUSTEXT)")
    for s in sim.statustext_kayit:
        log(f"   {s}")
    ok = any("HEDEF" in s for s in sim.statustext_kayit)
    gecti &= ok
    log(f"   [{'OK ' if ok else 'HATA'}] tespit kaniti gonderildi")

    if args.senaryo == "devralma":
        log("\n5) PILOT DEVRALMA DAVRANISI")
        if sim.pilot_devraldi_t is None:
            log("   [HATA] senaryo hic tetiklenmedi")
            gecti = False
        else:
            # Kriter: devralmadan sonra HICBIR setpoint gonderilmemeli.
            # 0.5 sn tolerans, o an ucusta olan bir paket icin.
            gecikme = sim.son_setpoint_t - sim.pilot_devraldi_t
            ok = gecikme < 0.5
            gecti &= ok
            log(f"   [{'OK ' if ok else 'HATA'}] devralmadan sonra son setpoint "
                f"{gecikme:+.2f} sn (0.5 sn'den kucuk olmali)")
            # Yazilim devralmayi fark etmis olmali
            fark_etti = any("PILOT DEVRALDI" in x for x in sim.statustext_kayit)
            gecti &= fark_etti
            log(f"   [{'OK ' if fark_etti else 'HATA'}] yazilim devralmayi "
                f"fark edip yer istasyonuna bildirdi")
            # Ve kendiliginden AUTO'ya donmeye CALISMAMALI
            zorlamadi = sim.mod == "LOITER"
            gecti &= zorlamadi
            log(f"   [{'OK ' if zorlamadi else 'HATA'}] mod hala LOITER "
                f"(yazilim modu geri almaya calismadi): {sim.mod}")

    log("\n" + ("PROVA GECTI" if gecti else "PROVA BASARISIZ"))
    return 0 if gecti else 1


if __name__ == "__main__":
    raise SystemExit(main())
