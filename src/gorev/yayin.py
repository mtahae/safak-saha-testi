"""
ŞAFAK UAV - Yer İstasyonuna Canlı Kanıt Yayını
===============================================
Şartname (10.2.2): "Takımlar, görüntü işleme yöntemiyle hedefi tespit ettiklerini,
yer kontrol istasyonu aracılığıyla uçuş hakemlerine kanıtlamakla yükümlüdür."
Bu yapılmazsa isabet olsa bile hedef isabet puanı SIFIRDIR. Yani bu modül
süs değil, puanın ön şartıdır.

NEDEN GSTREAMER DEĞİL: GStreamer donanımsal H.264 daha az bant genişliği
kullanır ama RPi'de kurulum/pipeline sorunları saatler yiyebilir ve sahada
çalışmazsa yedeği yoktur. Burada yalnızca Python standart kütüphanesi ve
OpenCV'nin JPEG kodlayıcısı kullanılır: kurulum yok, bağımlılık yok, bozulacak
bir şey yok. Yarışma alanı ölçeğinde (birkaç yüz metre Wi-Fi) fazlasıyla yeterli.

Protokol (basit, kayıp toleranslı):
    Her kare JPEG'e kodlanır ve 1400 baytlık parçalara bölünür.
    Her UDP paketi:  [kare_no:4][parca_no:2][parca_sayisi:2][veri...]
    Alıcı, parçaları kare numarasına göre birleştirir; eksik kalan kareyi atar.
    UDP olduğu için kaybolan paket akışı DURDURMAZ, sadece o kare düşer.
"""
import socket
import struct
import threading
import time

import cv2

BASLIK = struct.Struct("!IHH")      # kare_no, parca_no, parca_sayisi
MAKS_YUK = 1400                     # MTU'ya güvenli sığar (parçalanma olmaz)


class Yayin:
    def __init__(self, hedef_ip, port, log=print, kalite=55, maks_genislik=960,
                 hedef_fps=12):
        self.adres = (hedef_ip, port)
        self.log = log
        self.kalite = kalite
        self.maks_genislik = maks_genislik
        self.aralik = 1.0 / hedef_fps
        self.sok = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sok.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self._dur = False
        self._is = None
        self.gonderilen = 0

    def basla(self, kare_al):
        """kare_al: çağrıldığında son annotated kareyi döndüren fonksiyon."""
        self._is = threading.Thread(target=self._dongu, args=(kare_al,), daemon=True)
        self._is.start()
        self.log(f"[yayin] baslatildi -> {self.adres[0]}:{self.adres[1]}")

    def _dongu(self, kare_al):
        kare_no = 0
        while not self._dur:
            t0 = time.time()
            kare = kare_al()
            if kare is None:
                time.sleep(0.05)
                continue
            try:
                self._gonder(kare, kare_no)
                kare_no = (kare_no + 1) & 0xFFFFFFFF
                self.gonderilen += 1
            except Exception as e:
                # Yayın hatası göreve ASLA engel olmamalı: yer istasyonu
                # bağlantısı kopsa bile uçuş ve bırakma devam etmeli.
                self.log(f"[yayin] hata (gorev etkilenmez): {e}")
                time.sleep(0.5)
            uyku = self.aralik - (time.time() - t0)
            if uyku > 0:
                time.sleep(uyku)

    def _gonder(self, kare, kare_no):
        h, w = kare.shape[:2]
        if w > self.maks_genislik:
            o = self.maks_genislik / w
            kare = cv2.resize(kare, (self.maks_genislik, int(h * o)))
        ok, buf = cv2.imencode(".jpg", kare,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.kalite])
        if not ok:
            return
        veri = buf.tobytes()
        toplam = (len(veri) + MAKS_YUK - 1) // MAKS_YUK
        for i in range(toplam):
            parca = veri[i * MAKS_YUK:(i + 1) * MAKS_YUK]
            self.sok.sendto(BASLIK.pack(kare_no, i, toplam) + parca, self.adres)

    def durdur(self):
        self._dur = True
        try:
            self.sok.close()
        except Exception:
            pass
