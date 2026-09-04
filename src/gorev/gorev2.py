"""
ŞAFAK UAV - GÖREV 2 Otonom Görev Yazılımı
==========================================
Teknofest 2026 Liseler Arası İHA - Döner Kanat Kategorisi, İkinci Görev.

AKIŞ
----
    1. BEKLE      : Kumandadan arm + AUTO. Görev (waypoint'ler) uçuş
                    kontrolcüsüne önceden yüklenmiştir.
    2. YAKLASMA   : AUTO görevi ilerler. Tespit KAPALI.
                    (Şartname: hedef tespiti 2. direk dıştan alındıktan SONRA.)
    3. TARAMA     : TESPIT_ACILIS_WP'ye ulaşıldı -> tespit AÇIK.
                    Her kare GPS'e çevrilip havuza yazılır.
    4. BIRAKMA    : Hedef doğrulanınca AUTO duraklatılır (WP no saklanır),
                    GUIDED'a geçilir, hedefin üstüne inilir, servo tetiklenir.
    5. DEVAM      : AUTO'ya dönülür, saklanan waypoint'ten devam edilir.
                    Eksik hedef varsa tarama sürer (3'e dön).
    6. BITIS      : Görev bitiş çizgisini geçer, AUTO görevindeki iniş
                    komutuyla otonom iner. Kumandaya hiç dokunulmaz -> c=1.0

Kullanım:
    # SITL (donanımsız test)
    python -m src.gorev.gorev2 --baglanti tcp:127.0.0.1:5763 --kaynak video.mp4
    # Sahada (RPi5)
    python -m src.gorev.gorev2 --kaynak picam
    # Kuru çalışma (servo tetiklenmez, uçuş komutu gönderilmez)
    python -m src.gorev.gorev2 --prova
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

# Paket olarak da (python -m src.gorev.gorev2) düz script olarak da çalışsın
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
    from src.gorev import geo, gorev_config as cfg, ucus as ucus_mod
    from src.gorev.ucus import PilotDevraldi, BaglantiKayip
    from src.gorev.algilayici import Algilayici, Kamera
    from src.gorev.hedef_havuzu import HedefHavuzu
    from src.gorev.yayin import Yayin
else:
    from . import geo, gorev_config as cfg, ucus as ucus_mod
    from .ucus import PilotDevraldi, BaglantiKayip
    from .algilayici import Algilayici, Kamera
    from .hedef_havuzu import HedefHavuzu
    from .yayin import Yayin


class Kayitci:
    """Ekrana ve dosyaya aynı anda yazar. Uçuş sonrası kanıt/analiz için."""

    def __init__(self, dizin):
        dizin.mkdir(parents=True, exist_ok=True)
        self.yol = dizin / f"gorev2_{time.strftime('%Y%m%d_%H%M%S')}.log"
        self.f = open(self.yol, "a", encoding="utf-8", buffering=1)
        self.t0 = time.time()

    def __call__(self, *a):
        s = f"[{time.time()-self.t0:7.1f}s] " + " ".join(str(x) for x in a)
        print(s, flush=True)
        self.f.write(s + "\n")


class Gorev2:
    def __init__(self, args, log, kamera=None):
        """kamera: dışarıdan enjekte edilebilir (prova koşumu sahte kamera verir).
        None ise args.kaynak'tan gerçek kamera açılır."""
        self.args = args
        self.log = log
        self.prova = args.prova
        self.t_baslangic = None
        self.faz = "BASLIYOR"          # arayuz bunu okur

        # --- SAHA TESTI BAYRAKLARI ---------------------------------------
        # Ikisi de VARSAYILAN OLARAK KAPALI. Yarisma ucusunda hicbiri
        # verilmez; yalnizca eksik donanimla saha denemesi icin vardir.
        # Hangisi aciksa log'a ve arayuze BUYUK harfle basilir ki yanlislikla
        # acik unutulmus bir bayrakla ucusa cikilmasin.
        self.sadece_mavi = getattr(args, "sadece_mavi", False)
        self.temsili_servo = getattr(args, "temsili_servo", False)
        # Hangi hedefler aranacak? --sadece-mavi verilirse kirmizi hic
        # denenmez (kirmizi brandanin olmadigi ya da zeminin kirmizi oldugu
        # testlerde yanlis pozitif pesinde kosmayalim diye).
        self.birakma_sirasi = (["mavi_hedef"] if self.sadece_mavi
                               else list(cfg.BIRAKMA_SIRASI))

        self.kam_modeli = geo.KameraModeli(
            hfov_derece=cfg.KAMERA_HFOV_DERECE,
            montaj_yaw_derece=cfg.KAMERA_MONTAJ_YAW_DERECE,
            egim_duzeltme_derece=cfg.KAMERA_EGIM_DUZELTME_DERECE,
            yatis_duzeltme_derece=cfg.KAMERA_YATIS_DUZELTME_DERECE,
            vfov_derece=cfg.KAMERA_VFOV_DERECE,
        )
        self.havuz = HedefHavuzu(cfg.KUME_YARICAP_M, cfg.MIN_TESPIT_SAYISI, log)
        cfg.KANIT_DIZIN.mkdir(parents=True, exist_ok=True)
        self.kanit_no = 0
        self.birakma_kayit = []      # her bırakmanın ölçülmüş kaydı (rapor için)

        self.ucus = ucus_mod.Ucus(args.baglanti, args.baud,
                                  cfg.KENDI_SYSTEM_ID, cfg.KENDI_COMPONENT_ID, log,
                                  baglanti_kayip_s=cfg.BAGLANTI_KAYIP_S)
        self.ucus.hazir_bekle()

        self.ded = self._dedektor_sec()
        if getattr(self.ded, "kamera_de_saglar", False):
            # Hailo yolu: kamerayi GStreamer boru hatti tutuyor. Ikinci bir
            # Kamera acmak ayni cihazi iki kez acmak demektir; ikisi de calismaz.
            self.kamera = self.ded
            self.log("[gorev] kamera Hailo boru hattindan geliyor")
        else:
            self.kamera = kamera or Kamera(args.kaynak, cfg.KAMERA_GENISLIK,
                                           cfg.KAMERA_YUKSEKLIK, cfg.KAMERA_FPS, log)
        self.yayin = Yayin(cfg.YAYIN_HEDEF_IP, cfg.YAYIN_PORT, log) \
            if (cfg.YAYIN_ACIK and not args.yayin_yok) else None

        self.algi = Algilayici(self.ded, self.kamera, self.ucus, self.havuz,
                               self.kam_modeli, cfg, log, self._kanit_yaz)

    # ------------------------------------------------------------------
    def _dedektor_sec(self):
        """RPi'de NCNN, PC'de PyTorch. Elde ne varsa onu kullan.

        Hailo ASLA otomatik secilmez -- yalnizca `--motor hailo` ile. Sebep:
        Hailo yolu kamerayi da devraliyor ve bu, sahada dogrulanmadan
        varsayilan yapilamayacak kadar buyuk bir davranis degisikligi.
        """
        if self.args.motor == "hailo":
            from src.detect_hailo import HailoKaynak
            self.log("[gorev] dedektor: HAILO NPU (kamera da boru hattindan)")
            return HailoKaynak(cfg.HAILO_HEF, cfg.HAILO_ETIKET_JSON,
                               kaynak=cfg.HAILO_KAYNAK, conf=cfg.MIN_GUVEN,
                               kare_hizi=cfg.KAMERA_FPS, log=self.log)
        if self.args.motor in ("ncnn", "oto") and cfg.NCNN_MODEL_DIZIN.exists():
            try:
                from src.detect_ncnn import HedefDedektoruNCNN
                self.log("[gorev] dedektor: NCNN")
                return HedefDedektoruNCNN(cfg.NCNN_MODEL_DIZIN, conf=cfg.MIN_GUVEN)
            except Exception as e:
                self.log(f"[gorev] NCNN yuklenemedi ({e}), PyTorch deneniyor")
        from src.detect import HedefDedektoru
        self.log("[gorev] dedektor: PyTorch")
        return HedefDedektoru(cfg.PT_MODEL, conf=cfg.MIN_GUVEN)

    def _kanit_yaz(self, kare, sonuc, kume, d):
        """Bir hedef doğrulandığı anın karesini diske yazar (hakem kanıtı)."""
        self.kanit_no += 1
        ciz = self.ded.ciz(kare, sonuc)
        klat, klon = kume.konum
        cv2.putText(ciz, f"{kume.sinif} {klat:.7f},{klon:.7f} n={kume.sayi}",
                    (10, ciz.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 0), 4)
        cv2.putText(ciz, f"{kume.sinif} {klat:.7f},{klon:.7f} n={kume.sayi}",
                    (10, ciz.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2)
        yol = cfg.KANIT_DIZIN / f"kanit_{self.kanit_no:02d}_{kume.sinif}.jpg"
        cv2.imwrite(str(yol), ciz)
        self.log(f"[kanit] {yol.name}")
        if cfg.STATUSTEXT_ACIK:
            kisa = "K" if kume.sinif == "kirmizi_hedef" else "M"
            self.ucus.statustext(f"SAFAK HEDEF {kisa} {klat:.6f} {klon:.6f}")

    # ------------------------------------------------------------------
    def on_kontrol(self):
        d = self.ucus.durum()
        self.log("=" * 62)
        self.log("ON KONTROL")
        self.log(f"  mod          : {d.mod}")
        self.log(f"  GPS fix      : {d.gps_fix}  uydu: {d.uydu}")
        self.log(f"  irtifa (AGL) : {d.alt_agl:.1f} m")
        self.log(f"  lidar        : {d.lidar_m if d.lidar_m else 'YOK (barometre kullanilacak)'}")
        self.log(f"  batarya      : {d.batarya_v:.2f} V")
        self.log(f"  kamera HFOV  : {cfg.KAMERA_HFOV_DERECE} deg, "
                 f"montaj yaw {cfg.KAMERA_MONTAJ_YAW_DERECE} deg")
        self.log(f"  tespit WP    : {cfg.TESPIT_ACILIS_WP} "
                 f"(bu WP'ye ulasinca tespit acilir)")
        self.log(f"  servo        : kirmizi_yuk=k{cfg.SERVO_KANAL_KIRMIZI_YUK} "
                 f"mavi_yuk=k{cfg.SERVO_KANAL_MAVI_YUK}")
        gsd = geo.yer_ornekleme_m(cfg.YAKLASMA_IRTIFA_M, cfg.KAMERA_GENISLIK,
                                  self.kam_modeli)
        self.log(f"  GSD @{cfg.YAKLASMA_IRTIFA_M:.0f}m : {gsd*100:.1f} cm/piksel")
        if self.sadece_mavi:
            self.log("  *** SADECE MAVI: kirmizi hedef ARANMIYOR ***")
        if self.temsili_servo:
            self.log("  *** TEMSILI SERVO: servo komutu GONDERILMEYECEK ***")
        if self.prova:
            self.log("  *** PROVA MODU: servo ve ucus komutlari GONDERILMEYECEK ***")
        else:
            # Servolari bilinen KILITLI konuma al. Aciliste cikislarda coplu bir
            # PWM kalmis olabilir; bu yuk daha yerdeyken dusmesine yol acar.
            if self.temsili_servo:
                self.log("  TEMSILI SERVO: kilitleme komutu da gonderilmedi")
            else:
                for kanal in (cfg.SERVO_KANAL_KIRMIZI_YUK, cfg.SERVO_KANAL_MAVI_YUK):
                    self.ucus.servo(kanal, cfg.SERVO_KILITLI_PWM)
                self.log("  servolar kilitli konuma alindi")
        self.log("=" * 62)
        if d.gps_fix < 3:
            self.log("!! UYARI: 3D GPS fix yok. Geolokasyon calismaz.")

    def gorev_basla_bekle(self):
        """Kumandadan arm + AUTO bekler. Görev süresi bu andan sayılır."""
        self.log("[gorev] arm + AUTO bekleniyor (kumandadan)...")
        while True:
            d = self.ucus.durum()
            if d.armed and d.mod == "AUTO":
                self.t_baslangic = time.time()
                self.faz = "GOREV BASLADI"
                self.log(f"[gorev] GOREV BASLADI. wp={d.wp_no}")
                if cfg.STATUSTEXT_ACIK:
                    self.ucus.statustext("SAFAK GOREV2 BASLADI")
                return
            time.sleep(0.3)

    @property
    def gecen(self):
        return time.time() - self.t_baslangic if self.t_baslangic else 0.0

    # ------------------------------------------------------------------
    def tespiti_bekle_ve_ac(self):
        """Şartname gereği tespit, 2. direk dıştan alındıktan sonra açılır."""
        hedef_wp = cfg.TESPIT_ACILIS_WP
        self.faz = f"WP {hedef_wp} BEKLENIYOR"
        self.log(f"[gorev] WP {hedef_wp} bekleniyor (tespit kapisi)...")
        while self.gecen < cfg.GOREV_SURE_LIMIT_S:
            d = self.ucus.durum()
            if d.wp_no >= hedef_wp:
                self.algi.tespit_acik = True
                self.faz = "TARAMA"
                self.log(f"[gorev] >>> TESPIT ACILDI (wp={d.wp_no}) <<<")
                if cfg.STATUSTEXT_ACIK:
                    self.ucus.statustext("SAFAK TESPIT ACIK")
                return True
            if not d.armed:
                self.log("[gorev] arac disarm oldu, gorev iptal")
                return False
            time.sleep(0.3)
        return False

    def hepsi_birakildi(self):
        return all(any(k.birakildi for k in self.havuz.dogrulanmis(s))
                   for s in self.birakma_sirasi)

    def birakilacak_var_mi(self):
        """Bırakmaya hazır, önceliği en yüksek hedefi döndürür."""
        for sinif in self.birakma_sirasi:
            k = self.havuz.en_iyi(sinif)
            if k is not None:
                return k
        return None

    def tara(self):
        """Tarama fazı. Bırakılacak hedef çıkınca ya da tarama bölgesi
        bitince döner."""
        while True:
            d = self.ucus.durum()
            if self.gecen > cfg.BIRAKMAYI_BIRAK_S:
                self.log("[gorev] sure butcesi doldu, birakma denemeleri kesiliyor")
                return None
            if not d.armed:
                return None
            if self.ucus.devralindi:
                self.log("[gorev] pilot devraldi, birakma denemesi yapilmayacak")
                return None
            if 0 < d.batarya_v < cfg.MIN_BATARYA_V:
                self.log(f"[gorev] batarya dusuk ({d.batarya_v:.1f} V < "
                         f"{cfg.MIN_BATARYA_V} V), birakma kesiliyor")
                return None
            k = self.birakilacak_var_mi()
            if k is not None:
                return k
            if (cfg.TESPIT_KAPANIS_WP is not None
                    and d.wp_no >= cfg.TESPIT_KAPANIS_WP
                    and not self.havuz.dogrulanmis()):
                self.log("[gorev] tarama bolgesi bitti, hedef bulunamadi")
                return None
            time.sleep(0.2)

    # ------------------------------------------------------------------
    def _hedef_gecerli_mi(self, kume):
        """Bu hedefin üstüne uçmak güvenli mi?

        Bozuk bir duruş/irtifa ölçümü hedefi yüzlerce metre öteye koyabilir.
        Bu kontroller olmadan araç oraya doğru uçar — saha dışına, hakemlerin
        üstüne ya da menzil dışına. Şüpheli bir hedefi ATLAMAK, kötü bir
        hedefe uçmaktan her zaman iyidir.
        """
        lat, lon = kume.konum
        if kume.dagilim_m > cfg.MAKS_KUME_DAGILIM_M:
            self.log(f"[birak] REDDEDILDI {kume.sinif}: olcum dagilimi "
                     f"{kume.dagilim_m:.2f} m > {cfg.MAKS_KUME_DAGILIM_M} m. "
                     f"Olcumler tutarsiz, konum guvenilmez.")
            return False
        d = self.ucus.durum()
        if d.lat is None:
            self.log("[birak] REDDEDILDI: kendi konumumuz bilinmiyor")
            return False
        mesafe = geo.mesafe_m(d.lat, d.lon, lat, lon)
        if mesafe > cfg.MAKS_HEDEF_MESAFE_M:
            self.log(f"[birak] REDDEDILDI {kume.sinif}: hedef {mesafe:.0f} m "
                     f"uzakta (sinir {cfg.MAKS_HEDEF_MESAFE_M:.0f} m). "
                     f"Muhtemelen bozuk bir olcum.")
            return False
        return True

    def yuk_birak(self, kume):
        """Bir hedefin üstüne gidip yükü bırakır. AUTO'yu duraklatır.

        Dönüş: True (bırakıldı) / False (atlandı veya iptal edildi).
        """
        sinif = kume.sinif
        yuk_ad, kanal = cfg.YUK_ESLEME[sinif]
        lat, lon = kume.konum
        donus_wp = self.ucus.durum().wp_no      # AUTO'ya dönüşte buradan devam

        self.faz = f"BIRAKMA: {sinif}"
        self.log("-" * 62)
        self.log(f"[birak] HEDEF: {sinif} -> {yuk_ad} (servo k{kanal})")
        self.log(f"[birak] konum: {lat:.7f},{lon:.7f} "
                 f"n={kume.sayi} dagilim={kume.dagilim_m:.2f}m")

        if not self._hedef_gecerli_mi(kume):
            kume.birakildi = True      # tekrar denenmesin; varsa diger hedefe gecilir
            return False

        # Ölçülmüş sistematik sapma varsa hedefi ona göre kaydır. Bu değerler
        # ilk gerçek atıştan sonra, yükün düştüğü yer metreyle ölçülerek
        # gorev_config.py'ye girilir.
        if cfg.BIRAKMA_OFSET_KUZEY_M or cfg.BIRAKMA_OFSET_DOGU_M:
            lat, lon = geo.oteleme_uygula(lat, lon, cfg.BIRAKMA_OFSET_KUZEY_M,
                                          cfg.BIRAKMA_OFSET_DOGU_M)
            self.log(f"[birak] birakma ofseti uygulandi "
                     f"({cfg.BIRAKMA_OFSET_KUZEY_M:+.2f}K, "
                     f"{cfg.BIRAKMA_OFSET_DOGU_M:+.2f}D)")

        self.log(f"[birak] AUTO wp={donus_wp} saklandi")
        if cfg.STATUSTEXT_ACIK:
            self.ucus.statustext(f"SAFAK BIRAKMA {sinif[:8]}")

        if self.prova:
            self.log("[birak] PROVA: gercek komut gonderilmedi")
            kume.birakildi = True
            return True

        try:
            if not self.ucus.mod_ayarla("GUIDED"):
                self.log("[birak] GUIDED'a gecilemedi, birakma iptal")
                return False
            # Bu andan itibaren mod bizim komutumuz olmadan degisirse
            # PilotDevraldi firlatilir ve komut gondermeyi ANINDA birakiriz.
            self.ucus.kontrolu_iste("GUIDED")

            # 1) Yaklasma irtifasinda hedefin ustune
            ok, mesafe = self.ucus.varis_bekle(
                lat, lon, cfg.YAKLASMA_IRTIFA_M, cfg.KONUM_TOLERANS_M,
                cfg.IRTIFA_TOLERANS_M, cfg.VARIS_ZAMAN_ASIMI_S)
            self.log(f"[birak] yaklasma {'TAMAM' if ok else 'ZAMAN ASIMI'} "
                     f"mesafe={mesafe:.2f}m")

            # 2) Birakma irtifasina alcal
            ok, mesafe = self.ucus.varis_bekle(
                lat, lon, cfg.BIRAKMA_IRTIFA_M, cfg.KONUM_TOLERANS_M,
                cfg.IRTIFA_TOLERANS_M, cfg.VARIS_ZAMAN_ASIMI_S)
            self.log(f"[birak] alcalma {'TAMAM' if ok else 'ZAMAN ASIMI'} "
                     f"mesafe={mesafe:.2f}m")

            # 3) Salinim sonsun
            time.sleep(cfg.DURULMA_SANIYE)

            # 4) SON DUZELTME - hedefin tam ustunde, alcaktan alinan taze
            #    olcumler taramada uzaktan alinanlardan cok daha isabetli.
            yeni_lat, yeni_lon = kume.son_konum(saniye=8.0)
            if cfg.BIRAKMA_OFSET_KUZEY_M or cfg.BIRAKMA_OFSET_DOGU_M:
                yeni_lat, yeni_lon = geo.oteleme_uygula(
                    yeni_lat, yeni_lon, cfg.BIRAKMA_OFSET_KUZEY_M,
                    cfg.BIRAKMA_OFSET_DOGU_M)
            kayma = geo.mesafe_m(lat, lon, yeni_lat, yeni_lon)
            if 0.3 < kayma < cfg.KUME_YARICAP_M:
                self.log(f"[birak] son duzeltme: {kayma:.2f}m kayma, "
                         f"tekrar konumlaniyor")
                ok, mesafe = self.ucus.varis_bekle(
                    yeni_lat, yeni_lon, cfg.BIRAKMA_IRTIFA_M,
                    cfg.KONUM_TOLERANS_M, cfg.IRTIFA_TOLERANS_M, 20.0)
                time.sleep(1.5)
                lat, lon = yeni_lat, yeni_lon

            # 5) SON YAKLASMA - hem konum hem hiz sarti. Sadece "vardim" demek
            #    yetmez: o anda arac hala hareket halinde olabilir ve konum
            #    hatasi toleransin tamami kadar olur. Bu adim, isabetin alt
            #    sinirini KONUM_TOLERANS_M'den SON_KONUM_TOLERANS_M'e cekiyor.
            hazir, mesafe, hiz = self.ucus.birakmaya_hazir_bekle(
                lat, lon, cfg.BIRAKMA_IRTIFA_M, cfg.SON_KONUM_TOLERANS_M,
                cfg.MAKS_BIRAKMA_HIZI, cfg.SON_YAKLASMA_ZAMAN_ASIMI_S)
            if hazir:
                self.log(f"[birak] son yaklasma TAMAM: mesafe={mesafe:.2f}m "
                         f"hiz={hiz:.2f}m/s")
            else:
                self.log(f"[birak] UYARI: son yaklasma sarti saglanamadi "
                         f"(mesafe={mesafe:.2f}m hiz={hiz:.2f}m/s), "
                         f"yine de birakiliyor")

            # 6) Birakma aninin gercek konum hatasi - puanin kaynagi bu sayi
            d = self.ucus.durum()
            hata = geo.mesafe_m(d.lat, d.lon, lat, lon)
            self.log(f"[birak] BIRAKMA ANI: irtifa={d.irtifa:.2f}m "
                     f"hedefe_yatay_hata={hata:.2f}m hiz={d.yer_hizi:.2f}m/s")
            self.birakma_kayit.append({
                "sinif": sinif, "yuk": yuk_ad, "kanal": kanal,
                "hedef_lat": lat, "hedef_lon": lon,
                "birakma_lat": d.lat, "birakma_lon": d.lon,
                "irtifa": d.irtifa, "hiz": d.yer_hizi, "hata_m": hata,
                "olcum_sayisi": kume.sayi, "dagilim_m": kume.dagilim_m,
                "zaman_s": self.gecen,
            })

            # 7) Servo
            if self.temsili_servo:
                # Mekanizma takili degil. Servo komutu GONDERILMEZ; bunun
                # disindaki her sey (hizalanma, alcalma, hiz sarti, kayit,
                # goreve donus) gercekte oldugu gibi calisir.
                self.log(f"[birak] TEMSILI SERVO: kanal {kanal} "
                         f"tetiklenmedi, birakildi sayiliyor")
                time.sleep(cfg.SERVO_ACIK_KALMA_S)
            else:
                self.ucus.servo(kanal, cfg.SERVO_ACIK_PWM)
                time.sleep(cfg.SERVO_ACIK_KALMA_S)
                self.ucus.servo(kanal, cfg.SERVO_KILITLI_PWM)
            kume.birakildi = True
            self.log(f"[birak] {yuk_ad} BIRAKILDI")
            if cfg.STATUSTEXT_ACIK:
                self.ucus.statustext(f"SAFAK YUK BIRAKILDI h={hata:.1f}m")

            # 8) Goreve don
            self.ucus.gorev_wp_ayarla(donus_wp)
            self.ucus.mod_ayarla("AUTO")
            self.ucus.kontrolu_birak()
            self.faz = "TARAMA"
            self.log(f"[birak] AUTO'ya donuldu (wp={donus_wp})")
            self.log("-" * 62)
            return True

        except PilotDevraldi as e:
            # Pilot kumandayi aldi. Hicbir komut gondermiyoruz - AUTO'ya
            # dondurmeye bile CALISMIYORUZ, cunku pilotun o an yaptigi sey
            # bizim bildigimizden onemlidir.
            self.log(f"[birak] !!! PILOT DEVRALDI: {e}")
            self.log("[birak] tum ucus komutlari durduruldu. "
                     "Tespit ve yayin devam ediyor.")
            if cfg.STATUSTEXT_ACIK:
                self.ucus.statustext("SAFAK PILOT DEVRALDI")
            return False
        except BaglantiKayip as e:
            self.log(f"[birak] !!! BAGLANTI KAYBI: {e}")
            self.log("[birak] setpoint gonderimi durduruldu; ArduPilot kendi "
                     "failsafe'ini uygulayacak.")
            self.ucus.kontrolu_birak()
            return False


    def calistir(self):
        self.on_kontrol()
        self.algi.start()
        if self.yayin:
            self.yayin.basla(self.algi.kare_al)
        if getattr(self.args, "ui", False):
            # Arayuz AYRI bir surec degil ayri bir IS PARCACIGI; ama gorev
            # akisina hicbir sekilde dokunmaz, yalnizca okur. Arayuz cokerse
            # gorev devam eder.
            try:
                from test_ui.sunucu import Arayuz
                self.arayuz = Arayuz(self, port=self.args.ui_port)
                self.arayuz.basla()
            except Exception as e:
                self.log(f"[gorev] arayuz baslatilamadi (gorev etkilenmez): {e}")

        self.gorev_basla_bekle()
        if not self.tespiti_bekle_ve_ac():
            self.log("[gorev] tespit acilamadi, cikiliyor")
            return

        while not self.hepsi_birakildi():
            kume = self.tara()
            if kume is None:
                break
            self.yuk_birak(kume)
            if self.ucus.devralindi:
                # Pilot devraldi: bir daha hicbir ucus komutu gonderme.
                # Tespit ve yayin surer -- gorüntü isleme kaniti hala degerli.
                self.log("[gorev] pilot kontrolde; gorev yazilimi izleme moduna gecti")
                break
            if self.gecen > cfg.BIRAKMAYI_BIRAK_S:
                break

        self.log("[gorev] birakma fazi bitti, gorev AUTO ile devam ediyor")
        self.log("[gorev] HEDEF HAVUZU:\n" + self.havuz.ozet())
        self.log(f"[gorev] elenen kare/tespit sayaclari: {self.algi.elenen}")

        # Aracın inişini/disarm'ını izle
        while self.ucus.durum().armed and self.gecen < cfg.GOREV_SURE_LIMIT_S + 120:
            time.sleep(1.0)
        self.log(f"[gorev] BITTI. toplam sure {self.gecen:.0f}s "
                 f"kare={self.algi.kare_sayisi} gecerli_tespit={self.algi.gecerli_tespit}")
        self.rapor_yaz()

    def rapor_yaz(self):
        """Uçuş sonrası özet rapor. Hakem itirazına ve kendi analizimize dayanak.

        Şartname, tespitin görüntü işlemeyle yapıldığının kanıtlanmasını
        zorunlu kılıyor. Canlı yayın ve STATUSTEXT anlık kanıt; bu dosya ise
        uçuştan sonra elde kalan, koordinatlı ve zaman damgalı kayıt.
        """
        yol = cfg.KANIT_DIZIN / f"rapor_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        sat = []
        sat.append("SAFAK UAV - GOREV 2 UCUS RAPORU")
        sat.append("=" * 62)
        sat.append(f"tarih           : {time.strftime('%Y-%m-%d %H:%M:%S')}")
        sat.append(f"gorev suresi    : {self.gecen:.0f} s "
                   f"(sinir {cfg.GOREV_SURE_LIMIT_S} s)")
        sat.append(f"islenen kare    : {self.algi.kare_sayisi}")
        sat.append(f"gecerli tespit  : {self.algi.gecerli_tespit}")
        sat.append(f"elenen          : {self.algi.elenen}")
        sat.append(f"pilot devraldi  : {'EVET' if self.ucus.devralindi else 'hayir'}")
        sat.append("")
        sat.append("TESPIT EDILEN HEDEFLER (goruntu isleme ile)")
        sat.append("-" * 62)
        sat.append(self.havuz.ozet())
        sat.append("")
        sat.append("YUK BIRAKMALARI")
        sat.append("-" * 62)
        if not self.birakma_kayit:
            sat.append("  (hic yuk birakilmadi)")
        for b in self.birakma_kayit:
            sat.append(f"  {b['sinif']} <- {b['yuk']} (servo k{b['kanal']})")
            sat.append(f"    hedef konumu    : {b['hedef_lat']:.7f}, {b['hedef_lon']:.7f}")
            sat.append(f"    birakma konumu  : {b['birakma_lat']:.7f}, {b['birakma_lon']:.7f}")
            sat.append(f"    yatay hata      : {b['hata_m']:.2f} m")
            sat.append(f"    birakma irtifasi: {b['irtifa']:.2f} m")
            sat.append(f"    birakma hizi    : {b['hiz']:.2f} m/s")
            sat.append(f"    olcum sayisi    : {b['olcum_sayisi']} "
                       f"(dagilim {b['dagilim_m']:.2f} m)")
            sat.append(f"    gorev zamani    : {b['zaman_s']:.0f} s")
            sat.append("")
        sat.append("AYARLAR")
        sat.append("-" * 62)
        for ad in ("KAMERA_HFOV_DERECE", "KAMERA_MONTAJ_YAW_DERECE",
                   "KAMERA_GENISLIK", "KAMERA_YUKSEKLIK", "TESPIT_ACILIS_WP",
                   "MIN_TESPIT_SAYISI", "KUME_YARICAP_M", "YAKLASMA_IRTIFA_M",
                   "BIRAKMA_IRTIFA_M", "BIRAKMA_OFSET_KUZEY_M",
                   "BIRAKMA_OFSET_DOGU_M", "SERVO_KANAL_KIRMIZI_YUK",
                   "SERVO_KANAL_MAVI_YUK"):
            sat.append(f"  {ad:26s} = {getattr(cfg, ad)}")
        metin = "\n".join(sat)
        yol.write_text(metin, encoding="utf-8")
        self.log("\n" + metin)
        self.log(f"[gorev] rapor yazildi: {yol}")

    def kapat(self):
        self.algi.durdur()
        if self.yayin:
            self.yayin.durdur()
        self.kamera.kapat()
        self.ucus.kapat()


def main():
    ap = argparse.ArgumentParser(description="SAFAK UAV Gorev 2 otonom yazilimi")
    ap.add_argument("--baglanti", default=cfg.BAGLANTI,
                    help="MAVLink adresi (orn. tcp:127.0.0.1:5763 veya /dev/serial0)")
    ap.add_argument("--baud", type=int, default=cfg.BAUD)
    ap.add_argument("--kaynak", default="0",
                    help="'picam' (RPi Cam 3), webcam indeksi ('0') veya video dosyasi")
    ap.add_argument("--motor", choices=["oto", "ncnn", "pt", "hailo"], default="oto",
                    help="hailo: AI HAT+ NPU (kamerayi da devralir, "
                         "once tools/hailo_dogrula.py ile dogrulayin)")
    ap.add_argument("--prova", action="store_true",
                    help="servo tetiklenmez, ucus komutu gonderilmez (kuru test)")
    ap.add_argument("--yayin-yok", action="store_true")

    # --- SAHA TESTI BAYRAKLARI (yarisma ucusunda HICBIRI verilmez) --------
    ap.add_argument("--sadece-mavi", action="store_true",
                    help="sadece 2x2 mavi hedef aranir; kirmizi hic denenmez "
                         "(kirmizi branda yokken ya da zemin kirmiziyken)")
    ap.add_argument("--temsili-servo", action="store_true",
                    help="servo komutu GONDERILMEZ, birakildi sayilir "
                         "(yuk mekanizmasi takili degilken)")
    ap.add_argument("--ui", action="store_true",
                    help="Flask canli izleme arayuzunu baslat")
    ap.add_argument("--ui-port", type=int, default=5000)
    args = ap.parse_args()

    log = Kayitci(cfg.KANIT_DIZIN)
    log(f"SAFAK UAV Gorev 2 | log: {log.yol}")
    g = None
    try:
        g = Gorev2(args, log)
        g.calistir()
    except KeyboardInterrupt:
        log("[gorev] kullanici durdurdu")
    except Exception as e:
        import traceback
        log(f"[gorev] HATA: {e}\n{traceback.format_exc()}")
    finally:
        if g:
            g.kapat()


if __name__ == "__main__":
    main()
