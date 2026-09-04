"""
ŞAFAK UAV - Algılama İş Parçacığı
==================================
Kamerayı sürekli okur, YOLO ile hedefleri bulur, her tespiti anlık uçuş
durumuyla birleştirip GPS koordinatına çevirir ve hedef havuzuna yazar.

Görev durum makinesinden AYRI bir iş parçacığında çalışır. Sebep: uçuş
komutları (varış bekleme, alçalma) saniyeler sürer; bu sırada kamera akışı
durmamalı — hem tespit fırsatı kaçmasın hem de hakeme giden canlı kanıt
yayını kesilmesin.

ZAMAN HİZASI: Bir kare yakalandığı ANIN uçuş durumu kullanılır. Kare işlendikten
sonraki durum DEĞİL. 10 m/s hızda 100 ms gecikme 1 metre yer hatası demektir.
"""
import threading
import time

import cv2

from . import geo


class Kamera:
    """RPi Camera Module 3 (picamera2) veya USB webcam (OpenCV) — aynı arayüz.

    Camera Module 3, Raspberry Pi OS Bookworm'da libcamera üzerinden çalışır ve
    cv2.VideoCapture ile AÇILMAZ. Bu yüzden önce picamera2 denenir; yoksa
    OpenCV'ye düşülür (webcam ve masaüstü testi için).
    """

    def __init__(self, kaynak=0, genislik=1280, yukseklik=720, fps=30, log=print):
        self.log = log
        self.picam = None
        self.cap = None
        self.genislik, self.yukseklik = genislik, yukseklik

        if kaynak == "picam":
            from picamera2 import Picamera2
            self.picam = Picamera2()
            # ÖNEMLİ: ScalerCrop varsayılanı tam sensör alanıdır; böylece
            # geolokasyonun dayandığı 66 derece HFOV geçerli kalır. Kırpılmış
            # bir mod seçilirse görüş açısı daralır ve tüm hesap sistematik kayar.
            yapi = self.picam.create_video_configuration(
                main={"size": (genislik, yukseklik), "format": "RGB888"},
                controls={"FrameRate": fps})
            self.picam.configure(yapi)
            self.picam.start()
            time.sleep(1.0)  # otomatik pozlama otursun
            self.log(f"[kamera] picamera2 {genislik}x{yukseklik}@{fps}")
        else:
            self.cap = cv2.VideoCapture(int(kaynak) if str(kaynak).isdigit() else kaynak)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, genislik)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, yukseklik)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            # Tampon 1 kare: eski kare işlemek, geolokasyonda zaman kayması demek
            try:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            if not self.cap.isOpened():
                raise RuntimeError(f"Kamera acilamadi: {kaynak}")
            self.log(f"[kamera] OpenCV kaynak={kaynak}")

    def oku(self):
        if self.picam is not None:
            kare = self.picam.capture_array()
            return True, cv2.cvtColor(kare, cv2.COLOR_RGB2BGR)
        return self.cap.read()

    def kapat(self):
        if self.picam is not None:
            self.picam.stop()
        if self.cap is not None:
            self.cap.release()


class Algilayici(threading.Thread):
    def __init__(self, dedektor, kamera, ucus, havuz, kam_modeli, cfg,
                 log=print, kanit_yaz=None):
        super().__init__(daemon=True)
        self.ded = dedektor
        self.kam = kamera
        self.ucus = ucus
        self.havuz = havuz
        self.kam_modeli = kam_modeli
        self.cfg = cfg
        self.log = log
        self.kanit_yaz = kanit_yaz

        self.tespit_acik = False      # görev durum makinesi açıp kapatır
        self._dur = False
        self._kilit = threading.Lock()
        self.son_kare = None          # yayın için annotated kare
        self.fps = 0.0
        self.kare_sayisi = 0
        self.gecerli_tespit = 0
        self.elenen = {"durum": 0, "egim": 0, "irtifa": 0, "uzak": 0,
                       "guven": 0, "renk": 0, "ufuk": 0, "cok_yuksek": 0}
        self._yukseklik_uyarildi = False

    def run(self):
        son_t = time.time()
        while not self._dur:
            ok, kare = self.kam.oku()
            if not ok:
                time.sleep(0.05)
                continue
            # Kareyi yakaladığımız ANIN uçuş durumu — sonrakinin değil.
            d = self.ucus.durum()
            t0 = time.time()

            sonuc = self.ded.isle(kare)
            self.kare_sayisi += 1

            if self.tespit_acik:
                self._isle_tespitler(kare, sonuc, d)

            ciz = self.ded.ciz(kare, sonuc)
            self._bilgi_bas(ciz, d)
            with self._kilit:
                self.son_kare = ciz

            dt = time.time() - t0
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(dt, 1e-3))
            son_t = time.time()

    def _isle_tespitler(self, kare, sonuc, d):
        cfg = self.cfg
        H, W = kare.shape[:2]

        # Uçuş durumu geolokasyon için uygun mu? Uygun değilse tespiti SAYMA —
        # kötü bir ölçüm, hiç ölçüm olmamasından daha zararlıdır.
        if not d.gecerli:
            self.elenen["durum"] += 1
            return
        irtifa = d.irtifa
        if irtifa is None or irtifa < cfg.MIN_IRTIFA_M:
            self.elenen["irtifa"] += 1
            return
        if irtifa > cfg.TESPIT_MAKS_IRTIFA_M:
            # Bu irtifada 1x1 m hedef ~35 pikselin altına düşer ve model onu
            # göremez (bkz. gorev_config TARAMA_IRTIFA_HEDEF_M ölçüm tablosu).
            # Tespitleri yine de kabul ediyoruz -- gelen bir tespit gerçektir --
            # ama rotanın irtifası yanlışsa bunu SESSİZCE yaşamamalıyız.
            self.elenen["cok_yuksek"] += 1
            if not self._yukseklik_uyarildi:
                self._yukseklik_uyarildi = True
                self.log(f"[algi] !! UYARI: tarama irtifasi {irtifa:.0f} m, "
                         f"tavan {cfg.TESPIT_MAKS_IRTIFA_M:.0f} m. 1x1 hedef "
                         f"kacirilabilir. Rota irtifasini dusurun.")
        egim = max(abs(d.roll), abs(d.pitch))
        if egim > (cfg.MAKS_EGIM_DERECE * 3.14159265 / 180.0):
            # Sert viraj/hızlanmada duruş hızla değişiyor; küçük bir zaman
            # kayması bile büyük yer hatasına dönüşür. Bu kareleri atla.
            self.elenen["egim"] += 1
            return

        for h in sonuc["tum"]:
            if h.guven < cfg.MIN_GUVEN:
                self.elenen["guven"] += 1
                continue
            if h.renk_orani < cfg.MIN_RENK_ORANI:
                self.elenen["renk"] += 1
                continue

            r = geo.piksel_gps(h.cx, h.cy, W, H, d.lat, d.lon, irtifa,
                               d.roll, d.pitch, d.yaw, self.kam_modeli)
            if r is None:
                self.elenen["ufuk"] += 1
                continue
            lat, lon, yatay = r
            if yatay > cfg.MAKS_YATAY_MESAFE_M:
                self.elenen["uzak"] += 1
                continue

            self.gecerli_tespit += 1
            yeni = self.havuz.ekle(h.sinif_isim, lat, lon, h.guven,
                                   h.renk_orani, irtifa)
            if yeni is not None:
                klat, klon = yeni.konum
                self.log(f"[algi] HEDEF DOGRULANDI: {yeni.sinif} "
                         f"{klat:.7f},{klon:.7f} n={yeni.sayi} "
                         f"dagilim={yeni.dagilim_m:.2f}m")
                if self.kanit_yaz:
                    self.kanit_yaz(kare, sonuc, yeni, d)

    def _bilgi_bas(self, ciz, d):
        """Yer istasyonuna giden karenin üstüne uçuş durumu bindirir.

        Hakeme kanıt olarak sunulacak görüntünün üstünde tespit kutusuyla
        BİRLİKTE konum/irtifa/zaman görünmeli — kutu tek başına "ne zaman,
        nerede" sorusunu cevaplamaz.
        """
        satirlar = [
            time.strftime("%H:%M:%S") + f"  SAFAK UAV  {self.fps:4.1f} FPS",
            f"mod={d.mod} irtifa={d.irtifa if d.irtifa else 0:.1f}m "
            f"uydu={d.uydu} bat={d.batarya_v:.1f}V",
            f"konum={d.lat:.6f},{d.lon:.6f}" if d.lat else "konum=YOK",
            f"TESPIT {'ACIK' if self.tespit_acik else 'KAPALI'}  "
            f"gecerli={self.gecerli_tespit}",
        ]
        y = 22
        for s in satirlar:
            cv2.putText(ciz, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 0), 4)
            cv2.putText(ciz, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 255) if self.tespit_acik else (200, 200, 200), 1)
            y += 24

    def kare_al(self):
        with self._kilit:
            return None if self.son_kare is None else self.son_kare.copy()

    def durdur(self):
        self._dur = True
