"""
ŞAFAK UAV - Hedef Havuzu
=========================
Tek karelik tespite ASLA güvenilmez. Bu modül, uçuş boyunca gelen tespitleri
yerdeki konumlarına göre kümeler ve yeterince tekrar eden kümeyi "doğrulanmış
hedef" ilan eder.

Neden gerekli:
  - Tek karelik yanlış pozitif (güneş parlaması, kırmızı bir çanta) elenir.
  - GPS/duruş gürültüsü ortalanır. Aynı hedefin 20 farklı kareden ölçümü,
    tek ölçüme göre belirgin daha isabetli bir merkez verir.
  - Aynı hedef defalarca görülür; bunları tek bir hedefe indirger.

Merkez hesabında ORTALAMA değil MEDYAN kullanılır: küme içine sızmış tek bir
sapkın ölçüm ortalamayı metrelerce kaydırır, medyanı kaydıramaz.
"""
import statistics
import time
from dataclasses import dataclass, field

from . import geo


@dataclass
class Olcum:
    lat: float
    lon: float
    guven: float
    renk_orani: float
    irtifa: float
    zaman: float


@dataclass
class Kume:
    sinif: str
    olcumler: list = field(default_factory=list)
    birakildi: bool = False

    @property
    def sayi(self):
        return len(self.olcumler)

    @property
    def konum(self):
        """Kümenin medyan konumu (lat, lon)."""
        return (statistics.median(o.lat for o in self.olcumler),
                statistics.median(o.lon for o in self.olcumler))

    def son_konum(self, saniye=8.0, min_olcum=3):
        """Son `saniye` içindeki ölçümlerin medyanı.

        Hedefin TAM ÜSTÜNDE alçalırken alınan ölçümler, taramada uzaktan ve
        eğik açıyla alınanlardan çok daha isabetlidir: hedef kare merkezine
        yakındır (mercek/duruş hatası orada en küçüktür) ve irtifa düşüktür
        (duruş hatasının yer karşılığı irtifayla orantılı). Bırakma anında bu
        taze ölçümleri kullanmak, tüm uçuşun medyanını kullanmaktan iyidir.

        Yeterli taze ölçüm yoksa tüm geçmişin medyanına düşer.
        """
        esik = time.time() - saniye
        taze = [o for o in self.olcumler if o.zaman >= esik]
        if len(taze) < min_olcum:
            return self.konum
        return (statistics.median(o.lat for o in taze),
                statistics.median(o.lon for o in taze))

    @property
    def ort_guven(self):
        return sum(o.guven for o in self.olcumler) / len(self.olcumler)

    @property
    def dagilim_m(self):
        """Ölçümlerin medyandan ortalama sapması (metre) — kalite göstergesi.

        Küçükse ölçümler tutarlı; büyükse ya hedef hareket ediyor ya da
        geolokasyon girdilerinden biri (irtifa/duruş) bozuk.
        """
        if len(self.olcumler) < 2:
            return 0.0
        mlat, mlon = self.konum
        return sum(geo.mesafe_m(mlat, mlon, o.lat, o.lon)
                   for o in self.olcumler) / len(self.olcumler)

    def ekle(self, o: Olcum, maks_olcum=200):
        self.olcumler.append(o)
        if len(self.olcumler) > maks_olcum:
            # en eskiyi değil, en DÜŞÜK güvenlisini at — kalite korunur
            self.olcumler.remove(min(self.olcumler, key=lambda x: x.guven))


class HedefHavuzu:
    def __init__(self, kume_yaricap_m=4.0, min_tespit=5, log=print):
        self.yaricap = kume_yaricap_m
        self.min_tespit = min_tespit
        self.kumeler = []
        self.log = log
        self._duyurulan = set()

    def ekle(self, sinif, lat, lon, guven, renk_orani, irtifa):
        """Bir tespiti havuza koyar. Yeni doğrulanan hedef varsa onu döndürür."""
        o = Olcum(lat, lon, guven, renk_orani, irtifa, time.time())
        hedef_kume = None
        en_yakin = float("inf")
        for k in self.kumeler:
            if k.sinif != sinif:
                continue
            klat, klon = k.konum
            m = geo.mesafe_m(klat, klon, lat, lon)
            if m < self.yaricap and m < en_yakin:
                en_yakin, hedef_kume = m, k

        if hedef_kume is None:
            hedef_kume = Kume(sinif=sinif)
            self.kumeler.append(hedef_kume)
        hedef_kume.ekle(o)

        # Yeni doğrulandı mı?
        if (hedef_kume.sayi >= self.min_tespit
                and id(hedef_kume) not in self._duyurulan):
            self._duyurulan.add(id(hedef_kume))
            return hedef_kume
        return None

    def dogrulanmis(self, sinif=None):
        """Doğrulanmış kümeler; en çok tespit alan önce."""
        k = [c for c in self.kumeler if c.sayi >= self.min_tespit]
        if sinif:
            k = [c for c in k if c.sinif == sinif]
        return sorted(k, key=lambda c: c.sayi, reverse=True)

    def en_iyi(self, sinif):
        """Bir sınıf için en güvenilir, henüz yük bırakılmamış hedef."""
        adaylar = [c for c in self.dogrulanmis(sinif) if not c.birakildi]
        return adaylar[0] if adaylar else None

    def ozet(self):
        satirlar = []
        for k in sorted(self.kumeler, key=lambda c: c.sayi, reverse=True):
            lat, lon = k.konum
            durum = "ONAYLI" if k.sayi >= self.min_tespit else "bekliyor"
            if k.birakildi:
                durum = "BIRAKILDI"
            satirlar.append(
                f"  {k.sinif:14s} {durum:10s} n={k.sayi:3d} "
                f"guven={k.ort_guven:.2f} dagilim={k.dagilim_m:.2f}m "
                f"{lat:.7f},{lon:.7f}")
        return "\n".join(satirlar) if satirlar else "  (hic tespit yok)"
