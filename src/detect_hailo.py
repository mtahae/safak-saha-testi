"""
ŞAFAK UAV - Hailo-8/8L NPU Dedektörü (Raspberry Pi AI HAT+)
============================================================
Dördüncü çıkarım motoru. Diğer üçüyle (detect / detect_onnx / detect_ncnn)
AYNI sözleşmeyi sunar: `isle(bgr)` -> {"kirmizi_hedef", "mavi_hedef", "tum",
"kare_boyut"}. Görev katmanı hangi motorun kullanıldığını bilmez.

NEDEN BU MODÜL HEM KAMERA HEM DEDEKTÖR
---------------------------------------
Hailo'nun Python arayüzü bir GStreamer boru hattıdır ve kamerayı KENDİ açar:
kaynak -> ölçekleme -> NPU çıkarımı -> callback. Yani "kareyi ver, tespiti al"
diyebileceğiniz bir fonksiyon yok; boru hattı çalışır ve her kare için bizim
callback'imizi çağırır (İTME modeli). Bizim `Algilayici` ise ÇEKME modeliyle
çalışır: `kam.oku()` sonra `ded.isle(kare)`.

Bu modül ikisini birbirine bağlar: boru hattını arka planda çalıştırır, her
karede (kare + tespitler) ÇİFTİNİ saklar, ve dışarıya hem `Kamera` hem
`Dedektor` arayüzü sunar. Böylece algilayici.py ve gorev2.py'de akış değişmez.

    kaynak = HailoKaynak(...)
    gorev2 -> kamera = kaynak,  dedektor = kaynak

KARE/TESPİT EŞLEŞMESİ
---------------------
`oku()` ile `isle()` ardışık çağrılır ve İKİSİ DE Algilayici'nin tek iş
parçacığından gelir. `oku()` o anki çifti mandallar, `isle()` mandalladığı
sonucu döndürür. Böylece kare N'in görüntüsüyle kare N+1'in tespiti asla
karışmaz — karışsaydı geolokasyon sessizce yanlış koordinat üretirdi.

ÜÇ TUZAK — hepsi sessizce yanlış sonuç üretir, hata vermez
-----------------------------------------------------------
1) ETİKETLER: `--labels-json` verilmezse Hailo COCO'nun 80 sınıfını kullanır
   ve modelimiz "person"/"bicycle" döndürür. Dahası etiket listesinin ilk
   elemanı "unlabeled" dolgusudur; unutulursa sınıflar bir kayar ve KIRMIZI
   ile MAVİ yer değiştirir. Bu, her iki hedefte de yanlış yük = 0 puan
   demektir. Bu yüzden aşağıda etiket -> sınıf eşlemesi AÇIKÇA yapılır ve
   tanınmayan her etiket log'a basılır; sessizce yutulmaz.

2) RENK DÜZENİ: `get_numpy_from_buffer` RGB döndürür. `color_verify` BGR
   bekler. Çevirmezsek HSV'de kırmızı ile mavi yer değiştirir ve renk
   doğrulama katmanı gerçek hedefleri elemeye başlar.

3) GÖRÜŞ AÇISI: Boru hattı kareyi modelin girişine ölçeklerken KIRPARSA
   gerçek yatay görüş açısı gorev_config.KAMERA_HFOV_DERECE'den dar olur ve
   tüm GPS projeksiyonu sistematik kayar. `tools/hailo_dogrula.py` bunu
   donanımda ölçer. Kırpma varsa HFOV düzeltilmeli.

Kurulum (RPi5 + AI HAT+):
    sudo apt install hailo-all
    # venv MUTLAKA --system-site-packages ile kurulmalı (gi, hailo apt'ten gelir)
"""
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config  # noqa: E402
import color_verify  # noqa: E402
from detect_onnx import Hedef, ciz  # noqa: E402  (çizim tüm motorlarda ortak)

import cv2  # noqa: E402


# Hailo etiket dizesi -> bizim sınıf kimliğimiz.
# models/safak_etiketler.json ile AYNI olmak zorundadır.
ETIKET_ESLEME = {
    "kirmizi_hedef": config.KIRMIZI,
    "mavi_hedef": config.MAVI,
}

BOS_SONUC = {"kirmizi_hedef": None, "mavi_hedef": None, "tum": [],
             "kare_boyut": (0, 0)}


class HailoKaynak:
    """Hailo GStreamer boru hattı; hem Kamera hem Dedektör arayüzü sunar.

    Kamera arayüzü : oku() -> (ok, bgr),  kapat()
    Dedektör arayüzü: isle(bgr) -> sonuc,  ciz(bgr, sonuc)
    """

    # gorev2.py buna bakip ayri bir Kamera acmaz -- kamerayi boru hatti tutar,
    # ikinci bir surec ayni cihazi acmaya calisirsa ikisi de basarisiz olur.
    kamera_de_saglar = True

    def __init__(self, hef_yolu=None, etiket_json=None, kaynak="rpi",
                 conf=None, renk_dogrula=True, kare_hizi=30, log=print):
        self.log = log
        self.conf = conf if conf is not None else config.GUVEN_ESIGI
        self.renk_dogrula = renk_dogrula

        hef_yolu = Path(hef_yolu or (config.MODEL_DIZIN / "safak_yolov8n.hef"))
        etiket_json = Path(etiket_json or (config.MODEL_DIZIN / "safak_etiketler.json"))
        if not hef_yolu.exists():
            raise FileNotFoundError(f"HEF bulunamadi: {hef_yolu}")
        if not etiket_json.exists():
            # Bu dosya olmadan Hailo COCO etiketlerini kullanir ve model
            # "person"/"bicycle" dondurur. Sessizce devam etmek, ucusta
            # hic tespit alamamak demektir.
            raise FileNotFoundError(
                f"Etiket dosyasi bulunamadi: {etiket_json}\n"
                "Bu dosya OLMADAN Hailo COCO'nun 80 sinifini kullanir ve "
                "modelimiz 'person'/'bicycle' dondurur.")

        # --- paylasilan durum ---
        self._kilit = threading.Lock()
        self._son_kare = None        # BGR
        self._son_sonuc = None
        self._son_no = -1
        self._mandal = None          # oku()'nun mandalladigi sonuc
        self._hazir = threading.Event()
        self._hata = None
        self._dur = False
        self.kare_sayisi = 0
        self.bilinmeyen_etiket = {}  # etiket -> kac kez goruldu

        self._boru_baslat(hef_yolu, etiket_json, kaynak, kare_hizi)

    # ------------------------------------------------------------------
    def _boru_baslat(self, hef_yolu, etiket_json, kaynak, kare_hizi):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst  # noqa: F401  (Gst.init boru hattinda yapilir)
        from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import (
            app_callback_class)
        from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import (
            GStreamerDetectionApp)

        kaynak_nesne = self

        class _Kullanici(app_callback_class):
            def __init__(self):
                super().__init__()
                self.kaynak = kaynak_nesne

        # GStreamerDetectionApp ayarlari sys.argv'den okur; baska yolu yok.
        # -u (--use-frame): callback'e numpy kare gelsin -- renk dogrulama,
        #   yayin ve kanit karesi icin ZORUNLU.
        # --disable-sync: ekran senkronu olmadan azami hizda calis (bassiz RPi).
        eski_argv = sys.argv[:]
        sys.argv = [eski_argv[0],
                    "--hef-path", str(hef_yolu),
                    "--labels-json", str(etiket_json),
                    "--input", str(kaynak),
                    "--use-frame",
                    "--disable-sync",
                    "--frame-rate", str(kare_hizi)]
        try:
            self._kullanici = _Kullanici()
            self._kullanici.use_frame = True
            self._app = GStreamerDetectionApp(_callback, self._kullanici)
        finally:
            sys.argv = eski_argv

        self._is = threading.Thread(target=self._kos, daemon=True)
        self._is.start()

        # Boru hattinin gercekten kare uretmeye basladigini dogrula. Sessizce
        # olu bir boru hatti ile ucusa cikmak, tespit yok demektir.
        if not self._hazir.wait(timeout=20.0):
            if self._hata:
                raise RuntimeError(f"Hailo boru hatti coktu: {self._hata}")
            raise TimeoutError(
                "Hailo boru hatti 20 saniyede kare uretmedi. "
                "Kontrol: AI HAT+ takili mi ('hailortcli fw-control identify'), "
                "kamera baska bir surec tarafindan tutuluyor mu, "
                f"kaynak '{kaynak}' dogru mu.")
        self.log(f"[hailo] boru hatti calisiyor: {hef_yolu.name} "
                 f"kaynak={kaynak} etiket={etiket_json.name}")

    def _kos(self):
        try:
            self._app.run()
        except Exception as e:            # boru hattı çökerse sessiz kalmasın
            self._hata = e
            self.log(f"[hailo] !! BORU HATTI HATASI: {e}")
            self._hazir.set()

    # ------------------------------------------------------------------
    def _kare_geldi(self, rgb, tespitler):
        """Callback'ten çağrılır. rgb: numpy RGB; tespitler: hailo nesneleri."""
        # RGB -> BGR. Bu satır olmadan color_verify kırmızıyı mavi sanar.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        sonuc = self._tespitleri_cevir(bgr, tespitler)
        with self._kilit:
            self._son_kare = bgr
            self._son_sonuc = sonuc
            self._son_no += 1
            self.kare_sayisi += 1
        self._hazir.set()

    def _tespitleri_cevir(self, bgr, tespitler):
        """Hailo tespitlerini bizim Hedef nesnelerimize çevirir.

        Hailo kutuları 0..1 aralığında normalize verir ve NMS'i kendi yapar;
        bu yüzden detect_onnx.son_isle (ham çıktı çözer) burada kullanılmaz.
        Renk doğrulaması ise AYNEN uygulanır — iki katmanlı güvenlik Hailo'da
        da geçerli olmalı, çünkü tam olarak yanlış etiketlemeyi yakalayan
        katman odur.
        """
        H, W = bgr.shape[:2]
        mx, my = W / 2.0, H / 2.0
        tum = []
        for t in tespitler:
            guven = float(t.get_confidence())
            if guven < self.conf:
                continue
            etiket = t.get_label()
            sid = ETIKET_ESLEME.get(etiket)
            if sid is None:
                # COCO etiketi geldi demektir -> --labels-json calismamis.
                n = self.bilinmeyen_etiket.get(etiket, 0) + 1
                self.bilinmeyen_etiket[etiket] = n
                if n == 1:
                    self.log(f"[hailo] !! BILINMEYEN ETIKET '{etiket}'. "
                             "labels-json yuklenmemis olabilir; model COCO "
                             "siniflarini donduruyor. Tespitler SAYILMIYOR.")
                continue

            kutu = t.get_bbox()
            x1, y1 = kutu.xmin() * W, kutu.ymin() * H
            x2, y2 = kutu.xmax() * W, kutu.ymax() * H
            roran = color_verify.renk_orani(bgr, (x1, y1, x2, y2), sid)
            if self.renk_dogrula and roran < config.RENK_DOGRULAMA_MIN_ORAN:
                continue                       # aldatıcı / yanlış etiket -> ele
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            tum.append(Hedef(sid, config.SINIF_ISIMLERI[sid], guven,
                             (x1, y1, x2, y2), cx, cy,
                             (cx - mx) / mx, (cy - my) / my,
                             int((x2 - x1) * (y2 - y1)), roran))

        en_iyi = {config.KIRMIZI: None, config.MAVI: None}
        for h in tum:
            m = en_iyi[h.sinif_id]
            if m is None or h.guven > m.guven:
                en_iyi[h.sinif_id] = h
        return {"kirmizi_hedef": en_iyi[config.KIRMIZI],
                "mavi_hedef": en_iyi[config.MAVI],
                "tum": tum, "kare_boyut": (W, H)}

    # ------------------------------------------------------------------
    # Kamera arayüzü
    # ------------------------------------------------------------------
    def oku(self):
        """Son kareyi döndürür ve ona ait tespit sonucunu MANDALLAR.

        Mandallama şart: `Algilayici` önce oku() sonra isle() çağırır. Arada
        boru hattı yeni bir kare üretirse, kareyle tespitin eşleşmesi bozulur
        ve geolokasyon yanlış koordinat üretir — sessizce.
        """
        with self._kilit:
            if self._son_kare is None:
                self._mandal = None
                return False, None
            self._mandal = self._son_sonuc
            return True, self._son_kare.copy()

    def kapat(self):
        self._dur = True
        try:
            self._app.shutdown()
        except Exception:
            try:
                from gi.repository import GLib  # noqa
                self._app.loop.quit()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Dedektör arayüzü
    # ------------------------------------------------------------------
    def isle(self, bgr=None):
        """Mandallanan sonucu döndürür.

        DİKKAT: `bgr` argümanı YOK SAYILIR. Çıkarım kareyle birlikte NPU'da
        zaten yapıldı; burada yeniden çalıştırılacak bir model yok. Argüman
        yalnızca diğer üç motorla imza uyumu için duruyor.
        """
        if self._mandal is not None:
            return self._mandal
        with self._kilit:
            return self._son_sonuc or dict(BOS_SONUC)

    def ciz(self, bgr, sonuc):
        return ciz(bgr, sonuc)


def _callback(pad, info, user_data):
    """GStreamer pad probe — her karede bir kez çalışır."""
    from gi.repository import Gst
    import hailo
    from hailo_apps.hailo_app_python.core.common.buffer_utils import (
        get_caps_from_pad, get_numpy_from_buffer)

    tampon = info.get_buffer()
    if tampon is None:
        return Gst.PadProbeReturn.OK
    kaynak = user_data.kaynak
    if kaynak._dur:
        return Gst.PadProbeReturn.OK

    bicim, gen, yuk = get_caps_from_pad(pad)
    if bicim is None or gen is None or yuk is None:
        return Gst.PadProbeReturn.OK
    try:
        rgb = get_numpy_from_buffer(tampon, bicim, gen, yuk)
        roi = hailo.get_roi_from_buffer(tampon)
        kaynak._kare_geldi(rgb, roi.get_objects_typed(hailo.HAILO_DETECTION))
    except Exception as e:
        kaynak.log(f"[hailo] callback hatasi: {e}")
    return Gst.PadProbeReturn.OK
