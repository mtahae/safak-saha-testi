"""
ŞAFAK UAV - Geolokasyon: Piksel -> GPS Koordinatı
==================================================
Kameradaki bir pikselin YERDE hangi enlem/boylama denk geldiğini hesaplar.

Girdi:
    (u, v)      : hedefin görüntüdeki piksel merkezi
    İHA konumu  : lat, lon (GLOBAL_POSITION_INT)
    irtifa      : alt_agl - yerden yükseklik, metre (relative_alt veya lidar)
    duruş       : roll, pitch, yaw - radyan (ATTITUDE)
Çıktı:
    hedefin lat, lon'u + İHA'ya yatay mesafesi

Yöntem (düz zemin varsayımı - yarışma sahası düz çim):
    1. Piksel -> kamera ışını (pinhole modeli, HFOV'dan türetilen odak uzaklığı)
    2. Kamera ışını -> gövde çerçevesi (montaj dönüşü + nadir bakış)
    3. Gövde -> NED (roll/pitch/yaw ile ZYX dönüşü)
    4. Işını yer düzlemiyle kes -> Kuzey/Doğu ötelemesi
    5. Öteleme -> enlem/boylam

Eksen tanımları:
    Kamera : x sağa, y aşağı (görüntü satır yönü), z optik eksen boyunca ileri
    Gövde  : x ileri (burun), y sağa (sağ kanat), z aşağı
    NED    : x Kuzey, y Doğu, z Aşağı

Doğrulama: `python src/gorev/geo.py` -> analitik olarak bilinen 8 senaryoyu test eder.
"""
import math
from dataclasses import dataclass

import numpy as np

# WGS84
_A = 6378137.0              # ekvator yarıçapı (m)
_F = 1.0 / 298.257223563    # basıklık
_E2 = _F * (2 - _F)         # birinci dışmerkezlik karesi


@dataclass
class KameraModeli:
    """Pinhole kamera + gövdeye montaj bilgisi."""
    hfov_derece: float = 66.0
    montaj_yaw_derece: float = 0.0      # kameranın gövdeye göre dönüşü (aşağı eksen etrafında)
    egim_duzeltme_derece: float = 0.0   # nadirden pitch sapması (+ ileri bakar)
    yatis_duzeltme_derece: float = 0.0  # nadirden roll sapması  (+ sağa bakar)
    # Dikey görüş açısı. None ise KARE PİKSEL varsayılır (fy = fx) ve dikey
    # açı kare yüksekliğinden kendiliğinden çıkar — normal kamera yolunda
    # doğru olan budur.
    #
    # NE ZAMAN DOLDURULUR: Görüntü boru hattı kareyi ANAMORFIK ölçeklerse,
    # yani en/boy oranını koruMADAN gerdiyse. Hailo/GStreamer yolu 16:9
    # sensör karesini 640x640 KAREYE gerebilir; o zaman fy != fx olur ve tek
    # odak uzaklığı kullanmak yatay ile dikeyi farklı ölçeklendirir. Sonuç
    # hata vermez: hedef sistematik olarak yanlış bir noktaya düşer ve sapma
    # eksen açısına göre değişir. tools/hailo_dogrula.py bunu ölçer.
    vfov_derece: float = None

    def odak_piksel(self, kare_genislik: int) -> float:
        """Yatay odak uzaklığı (fx), piksel cinsinden."""
        return (kare_genislik / 2.0) / math.tan(math.radians(self.hfov_derece) / 2.0)

    def odak_piksel_y(self, kare_genislik: int, kare_yukseklik: int) -> float:
        """Dikey odak uzaklığı (fy), piksel cinsinden.

        vfov_derece verilmemişse kare piksel varsayımıyla fx'e eşittir.
        """
        if self.vfov_derece is None:
            return self.odak_piksel(kare_genislik)
        return (kare_yukseklik / 2.0) / math.tan(math.radians(self.vfov_derece) / 2.0)


def _donus_govde_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Gövde -> NED dönüş matrisi (ZYX / 3-2-1 havacılık sırası). Açılar radyan."""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _piksel_isini_govde(u, v, kare_g, kare_y, kam: KameraModeli) -> np.ndarray:
    """Piksel -> gövde çerçevesinde birim olmayan yön vektörü (ileri, sağ, aşağı)."""
    fx = kam.odak_piksel(kare_g)
    fy = kam.odak_piksel_y(kare_g, kare_y)
    # Görüntü merkezine göre normalize ışın (pinhole)
    a = (u - kare_g / 2.0) / fx     # kamera x  (sağa)
    b = (v - kare_y / 2.0) / fy     # kamera y  (aşağı = görüntüde alta doğru)

    # Nadire bakan kamera, üst kenarı burna dönük:
    #   görüntüde YUKARI (b azalır) = gövdede İLERİ
    #   görüntüde SAĞA   (a artar)  = gövdede SAĞA
    #   optik eksen                 = gövdede AŞAĞI
    ileri, sag, asagi = -b, a, 1.0

    # Kameranın montaj dönüşü (aşağı ekseni etrafında)
    my = math.radians(kam.montaj_yaw_derece)
    if my:
        c, s = math.cos(my), math.sin(my)
        ileri, sag = ileri * c - sag * s, ileri * s + sag * c

    d = np.array([ileri, sag, asagi], dtype=float)

    # Mekanik montaj sapması (nadirden kaçıklık) — küçük açı düzeltmesi
    if kam.egim_duzeltme_derece or kam.yatis_duzeltme_derece:
        d = _donus_govde_ned(
            math.radians(kam.yatis_duzeltme_derece),
            math.radians(kam.egim_duzeltme_derece),
            0.0,
        ) @ d
    return d


def metre_basina_derece(lat_derece: float):
    """Verilen enlemde 1 derece enlem/boylamın kaç metre olduğunu döndürür.

    WGS84 elipsoidi üzerinden hesaplanır; sabit 111320 yaklaşımından farklı
    olarak enleme göre değişen meridyen yarıçapını kullanır.
    """
    lat = math.radians(lat_derece)
    s = math.sin(lat)
    tmp = 1.0 - _E2 * s * s
    m_meridyen = _A * (1 - _E2) / (tmp ** 1.5)   # kuzey-güney eğrilik yarıçapı
    n_dikey = _A / math.sqrt(tmp)                # doğu-batı eğrilik yarıçapı
    metre_lat = math.radians(1.0) * m_meridyen
    metre_lon = math.radians(1.0) * n_dikey * math.cos(lat)
    return metre_lat, metre_lon


def oteleme_uygula(lat: float, lon: float, kuzey_m: float, dogu_m: float):
    """Bir konuma kuzey/doğu metre ötelemesi ekleyip yeni lat/lon döndürür."""
    m_lat, m_lon = metre_basina_derece(lat)
    return lat + kuzey_m / m_lat, lon + dogu_m / m_lon


def mesafe_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """İki koordinat arası yatay mesafe (metre). Yarışma ölçeğinde (<1 km)
    düzlem yaklaşımı santimetre altı hata verir, haversine'e gerek yok."""
    m_lat, m_lon = metre_basina_derece((lat1 + lat2) / 2.0)
    dn = (lat2 - lat1) * m_lat
    de = (lon2 - lon1) * m_lon
    return math.hypot(dn, de)


def kuzey_dogu_farki(lat1: float, lon1: float, lat2: float, lon2: float):
    """1'den 2'ye kuzey/doğu bileşenleri (metre)."""
    m_lat, m_lon = metre_basina_derece((lat1 + lat2) / 2.0)
    return (lat2 - lat1) * m_lat, (lon2 - lon1) * m_lon


def piksel_gps(u, v, kare_g, kare_y,
               drone_lat, drone_lon, alt_agl,
               roll, pitch, yaw,
               kam: KameraModeli = None):
    """Bir pikselin yerdeki GPS karşılığını hesaplar.

    roll/pitch/yaw RADYAN (MAVLink ATTITUDE mesajıyla aynı birim).
    alt_agl METRE, yerden yükseklik.

    Dönüş: (lat, lon, yatay_mesafe_m)  ya da ışın yere değmiyorsa None.
    """
    if kam is None:
        kam = KameraModeli()
    if alt_agl is None or alt_agl <= 0:
        return None

    d_govde = _piksel_isini_govde(u, v, kare_g, kare_y, kam)
    d_ned = _donus_govde_ned(roll, pitch, yaw) @ d_govde

    asagi = d_ned[2]
    if asagi <= 1e-6:
        # Işın ufka paralel ya da yukarı bakıyor — yer düzlemini kesmiyor.
        return None

    t = alt_agl / asagi          # ışını yer düzlemine kadar uzat
    kuzey = t * d_ned[0]
    dogu = t * d_ned[1]

    lat, lon = oteleme_uygula(drone_lat, drone_lon, kuzey, dogu)
    return lat, lon, math.hypot(kuzey, dogu)


def gps_piksel(hedef_lat, hedef_lon, kare_g, kare_y,
               drone_lat, drone_lon, alt_agl,
               roll, pitch, yaw,
               kam: KameraModeli = None):
    """piksel_gps'in TERSİ: yerdeki bir koordinat görüntünün neresine düşer?

    Kullanım alanları:
      - Simülasyonla doğrulama (bilinen hedefi kareye yansıt, geri çöz, hatayı ölç)
      - Bilinen bir hedefin kadraja girip girmediğini önceden kestirmek

    Dönüş: (u, v, kadrajda_mi)  ya da hedef kameranın arkasındaysa None.
    """
    if kam is None:
        kam = KameraModeli()
    kuzey, dogu = kuzey_dogu_farki(drone_lat, drone_lon, hedef_lat, hedef_lon)
    v_ned = np.array([kuzey, dogu, float(alt_agl)])     # hedef bizden alt kadar AŞAĞIDA

    # NED -> gövde (dönüş matrisinin tersi = transpozu)
    v_govde = _donus_govde_ned(roll, pitch, yaw).T @ v_ned

    # Montaj sapma düzeltmesini geri al
    if kam.egim_duzeltme_derece or kam.yatis_duzeltme_derece:
        v_govde = _donus_govde_ned(
            math.radians(kam.yatis_duzeltme_derece),
            math.radians(kam.egim_duzeltme_derece), 0.0).T @ v_govde

    ileri, sag, asagi = v_govde
    if asagi <= 1e-6:
        return None                                     # hedef kameranın arkasında

    # Montaj yaw'ını geri al
    my = math.radians(kam.montaj_yaw_derece)
    if my:
        c, s = math.cos(my), math.sin(my)
        ileri, sag = ileri * c + sag * s, -ileri * s + sag * c

    fx = kam.odak_piksel(kare_g)
    fy = kam.odak_piksel_y(kare_g, kare_y)
    a = sag / asagi
    b = -ileri / asagi
    u = kare_g / 2.0 + a * fx
    v = kare_y / 2.0 + b * fy
    return u, v, (0 <= u < kare_g and 0 <= v < kare_y)


def yer_ornekleme_m(alt_agl: float, kare_g: int, kam: KameraModeli = None) -> float:
    """Nadir bakışta bir pikselin yerde kaç metreye denk geldiği (GSD).

    Geolokasyon hassasiyetinin alt sınırını verir: hedef merkezini 1 piksel
    şaşırırsan yerde bu kadar şaşırırsın.
    """
    if kam is None:
        kam = KameraModeli()
    return alt_agl / kam.odak_piksel(kare_g)


# ---------------------------------------------------------------------------
# Kendi kendini test — analitik olarak bilinen senaryolar
# ---------------------------------------------------------------------------
def _test():
    import sys
    G, Y = 1280, 720
    kam = KameraModeli(hfov_derece=66.0)
    f = kam.odak_piksel(G)
    lat0, lon0 = 39.9334, 32.8597    # Ankara civarı
    ALT = 30.0
    basarisiz = 0

    def kontrol(ad, olculen, beklenen, tolerans):
        nonlocal basarisiz
        fark = abs(olculen - beklenen)
        ok = fark <= tolerans
        if not ok:
            basarisiz += 1
        print(f"  [{'OK ' if ok else 'HATA'}] {ad}: {olculen:.3f} (beklenen {beklenen:.3f}, "
              f"fark {fark:.4f}, tol {tolerans})")

    print(f"Kamera: HFOV=66deg, {G}x{Y} -> f={f:.1f} px")
    print(f"VFOV   = {2*math.degrees(math.atan((Y/2)/f)):.1f} deg")
    print(f"GSD@{ALT:.0f}m = {yer_ornekleme_m(ALT, G, kam)*100:.1f} cm/piksel")
    print(f"Kare kapsamı @{ALT:.0f}m = {2*ALT*math.tan(math.radians(33)):.1f} x "
          f"{2*ALT*(Y/2)/f:.1f} m\n")

    # 1) Kare merkezi, düz uçuş -> tam altımız
    print("1) Merkez piksel, sifir attitude -> drone'un tam altinda")
    r = piksel_gps(G/2, Y/2, G, Y, lat0, lon0, ALT, 0, 0, 0, kam)
    kontrol("yatay mesafe", r[2], 0.0, 0.01)
    kontrol("enlem farki", abs(r[0]-lat0), 0.0, 1e-9)

    # 2) Sağ kenar, yaw=0 (kuzeye bakıyor) -> hedef tam DOĞUDA, alt*tan(HFOV/2)
    print("2) Sag kenar pikseli, yaw=0(kuzey) -> dogu ofseti = alt*tan(33deg)")
    r = piksel_gps(G, Y/2, G, Y, lat0, lon0, ALT, 0, 0, 0, kam)
    dn, de = kuzey_dogu_farki(lat0, lon0, r[0], r[1])
    kontrol("dogu ofseti", de, ALT * math.tan(math.radians(33)), 0.02)
    kontrol("kuzey ofseti", dn, 0.0, 0.01)

    # 3) Üst kenar (görüntüde yukarı = ileri), yaw=0 -> hedef KUZEYDE
    print("3) Ust kenar pikseli, yaw=0 -> kuzey ofseti pozitif")
    r = piksel_gps(G/2, 0, G, Y, lat0, lon0, ALT, 0, 0, 0, kam)
    dn, de = kuzey_dogu_farki(lat0, lon0, r[0], r[1])
    kontrol("kuzey ofseti", dn, ALT * (Y/2) / f, 0.02)
    kontrol("dogu ofseti", de, 0.0, 0.01)

    # 4) Aynı piksel ama yaw=90 (doğuya bakıyor) -> ileri artık DOĞU
    print("4) Ust kenar pikseli, yaw=90(dogu) -> ofset doguya doner")
    r = piksel_gps(G/2, 0, G, Y, lat0, lon0, ALT, 0, 0, math.radians(90), kam)
    dn, de = kuzey_dogu_farki(lat0, lon0, r[0], r[1])
    kontrol("dogu ofseti", de, ALT * (Y/2) / f, 0.02)
    kontrol("kuzey ofseti", dn, 0.0, 0.02)

    # 5) Merkez piksel, burun 10 derece AŞAĞI (pitch=-10).
    #    Burun aşağı inince kuyruk yukarı kalkar, karın ARKAYA yatar; nadire
    #    bakan kamera bu yüzden GERİYE bakar. Hedef GÜNEYDE çıkar (yaw=0 iken).
    print("5) Merkez piksel, pitch=-10deg (burun asagi) -> kamera geriye bakar (guney)")
    r = piksel_gps(G/2, Y/2, G, Y, lat0, lon0, ALT, 0, math.radians(-10), 0, kam)
    dn, de = kuzey_dogu_farki(lat0, lon0, r[0], r[1])
    kontrol("kuzey ofseti", dn, -ALT * math.tan(math.radians(10)), 0.02)

    # 6) Merkez piksel, roll=+10 (sağa yatık) -> hedef SOLDA (batıda)
    print("6) Merkez piksel, roll=+10deg (saga yatik) -> hedef batida")
    r = piksel_gps(G/2, Y/2, G, Y, lat0, lon0, ALT, math.radians(10), 0, 0, kam)
    dn, de = kuzey_dogu_farki(lat0, lon0, r[0], r[1])
    kontrol("dogu ofseti", de, -ALT * math.tan(math.radians(10)), 0.02)

    # 7) Attitude düzeltmesi çalışıyor mu: pitch=-10 ve kamera egim düzeltmesi
    #    +10 -> birbirini götürmeli, hedef tam altta.
    print("7) pitch=-10 + kamera egim duzeltmesi -10 -> birbirini goturur")
    kam2 = KameraModeli(hfov_derece=66.0, egim_duzeltme_derece=10.0)
    r = piksel_gps(G/2, Y/2, G, Y, lat0, lon0, ALT, 0, math.radians(-10), 0, kam2)
    kontrol("yatay mesafe", r[2], 0.0, 0.02)

    # 8) İrtifa iki katına çıkınca ofset de iki katına çıkmalı (lineerlik)
    print("8) Irtifa 2x -> ofset 2x (lineerlik)")
    r1 = piksel_gps(G, Y/2, G, Y, lat0, lon0, ALT, 0, 0, 0, kam)
    r2 = piksel_gps(G, Y/2, G, Y, lat0, lon0, ALT*2, 0, 0, 0, kam)
    kontrol("mesafe orani", r2[2] / r1[2], 2.0, 0.001)

    # 9) Ufkun üstüne çıkan ışın None dönmeli.
    #    pitch=-80 -> kamera neredeyse yatay geriye bakar; ALT kenar pikseli de
    #    gövdede geriye baktığı için ışın ufkun üstüne çıkar, yere değmez.
    print("9) Ufkun ustune cikan isin -> None")
    r = piksel_gps(G/2, Y, G, Y, lat0, lon0, ALT, 0, math.radians(-80), 0, kam)
    print(f"  [{'OK ' if r is None else 'HATA'}] sonuc: {r}")
    if r is not None:
        basarisiz += 1

    # 10) Hata duyarlılığı tablosu — sahada neyin ne kadar önemli olduğunu gösterir
    print("\n10) HATA DUYARLILIGI (30m irtifada, hedef kare merkezinde):")
    print(f"  attitude 1 derece hata      -> {ALT*math.tan(math.radians(1))*100:.0f} cm yer hatasi")
    print(f"  irtifa   1 m   hata         -> 0 cm (merkezde), kenarda "
          f"{1.0*math.tan(math.radians(33))*100:.0f} cm")
    print(f"  piksel   5 px  hata         -> {yer_ornekleme_m(ALT,G,kam)*5*100:.0f} cm")
    print(f"  GPS      1 m   hata         -> 100 cm (dogrudan aktarilir)")

    print(f"\n{'TUM TESTLER GECTI' if basarisiz==0 else f'{basarisiz} TEST BASARISIZ'}")
    return 0 if basarisiz == 0 else 1


if __name__ == "__main__":
    raise SystemExit(_test())
