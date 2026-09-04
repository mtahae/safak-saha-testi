"""
ŞAFAK UAV - ArduPilot Uçuş Arayüzü (pymavlink)
===============================================
Cube Orange+ / ArduPilot Copter ile konuşan ince katman. Görev durum makinesi
uçuş kontrolcüsünün detaylarını bilmek zorunda kalmasın diye her şey burada.

NEDEN MAVSDK DEĞİL: MAVSDK'nın "Offboard" modu bir PX4 kavramıdır. ArduPilot'ta
karşılığı GUIDED modudur ve MAVSDK'nın ArduPilot desteği eksiktir. pymavlink
doğrudan MAVLink konuşur, ArduPilot'un kendi belgelerindeki yol budur.

TASARIM: Arka planda bir okuma iş parçacığı sürekli telemetriyi tüketip son
durumu saklar. Görev döngüsü `durum()` ile anlık ve TUTARLI bir fotoğraf alır —
konum ile duruş açıları farklı anlara ait olmaz, çünkü geolokasyon buna çok
duyarlıdır (1 derece duruş hatası 30 m'de 52 cm yer hatası demek).
"""
import math
import threading
import time
from dataclasses import dataclass, field

from pymavlink import mavutil

# ArduPilot Copter uçuş modları (custom_mode değerleri)
MOD = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
    "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "BRAKE": 17,
}
MOD_ISIM = {v: k for k, v in MOD.items()}


class PilotDevraldi(Exception):
    """Pilot kumandadan kontrolü aldı — yazılım komut göndermeyi bırakmalı.

    Companion bilgisayarın pilotla kontrol için yarışması, çok rotorlu
    araçlarda en tehlikeli arıza kiplerinden biridir. Bu istisna fırlatıldığı
    anda görev yazılımı hiçbir uçuş komutu göndermez.
    """


class BaglantiKayip(Exception):
    """Uçuş kontrolcüsünden telemetri gelmiyor."""


@dataclass
class Durum:
    """İHA'nın tek bir andaki tutarlı fotoğrafı."""
    zaman: float = 0.0          # bu fotoğrafın alındığı yerel zaman (time.time)
    lat: float = None           # derece
    lon: float = None           # derece
    alt_agl: float = None       # yerden yükseklik, metre (relative_alt)
    alt_amsl: float = None      # deniz seviyesinden, metre
    roll: float = 0.0           # radyan
    pitch: float = 0.0          # radyan
    yaw: float = 0.0            # radyan
    yer_hizi: float = 0.0       # m/s
    mod: str = "?"
    armed: bool = False
    wp_no: int = -1             # AUTO görevinde o an hedeflenen waypoint
    lidar_m: float = None       # yere bakan mesafe sensörü (varsa)
    uydu: int = 0
    gps_fix: int = 0
    batarya_v: float = 0.0
    konum_yasi: float = 999.0   # konum verisinin kaç saniyelik olduğu
    durus_yasi: float = 999.0   # duruş verisinin kaç saniyelik olduğu

    @property
    def gecerli(self) -> bool:
        """Geolokasyon için kullanılabilir mi?"""
        return (self.lat is not None and self.alt_agl is not None
                and self.konum_yasi < 0.5 and self.durus_yasi < 0.5)

    @property
    def irtifa(self) -> float:
        """Geolokasyonda kullanılacak yerden yükseklik.

        Lidar varsa ve makul aralıktaysa o tercih edilir — barometrik irtifa
        gün içinde metrelerce kayabilir, lidar kaymaz.
        """
        if self.lidar_m is not None and 0.3 < self.lidar_m < 50.0:
            return self.lidar_m
        return self.alt_agl


class Ucus:
    def __init__(self, baglanti, baud=921600, sysid=191, compid=191, log=print,
                 baglanti_kayip_s=3.0):
        self.log = log
        self._kilit = threading.Lock()
        self._dur = False
        self._d = Durum()
        self._son_konum_t = 0.0
        self._son_durus_t = 0.0
        # Devralma gozetimi: yazilim kontrolu eline aldiginda beklenen modu
        # kaydeder; mod bizim komutumuz olmadan degisirse pilot devralmistir.
        self._beklenen_mod = None
        self._devralindi = False
        self._baglanti_kayip_s = baglanti_kayip_s
        # Komut onaylari (COMMAND_ACK) SADECE okuma is parcacigi tarafindan
        # toplanir. Iki is parcaciginin ayni baglantidan recv_match cagirmasi
        # guvenli degildir: onay mesajini digeri kapar ve komut, aslinda
        # basariyla islenmis olmasina ragmen "basarisiz" raporlanir.
        self._ack = {}

        self.log(f"[ucus] baglaniliyor: {baglanti}")
        self.mav = mavutil.mavlink_connection(
            baglanti, baud=baud, source_system=sysid, source_component=compid)
        # Kendimizi duyur: MAVLink'te her düğüm heartbeat yayınlar. Bu olmadan
        # karşı taraf bizi tanımaz (UDP'de dönüş adresimizi bile bilemez).
        self._heartbeat_gonder()
        self.mav.wait_heartbeat(timeout=30)
        if self.mav.target_system == 0:
            raise RuntimeError("Uçuş kontrolcüsünden heartbeat alınamadı")
        self.log(f"[ucus] baglandi. sys={self.mav.target_system} "
                 f"comp={self.mav.target_component}")

        self._is = threading.Thread(target=self._oku_dongusu, daemon=True)
        self._is.start()
        self._hb = threading.Thread(target=self._heartbeat_dongusu, daemon=True)
        self._hb.start()
        self.veri_akisi_iste()

    # ------------------------------------------------------------------
    # Telemetri
    # ------------------------------------------------------------------
    def _heartbeat_gonder(self):
        try:
            self.mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)
        except Exception:
            pass

    def _heartbeat_dongusu(self):
        while not self._dur:
            self._heartbeat_gonder()
            time.sleep(1.0)

    def veri_akisi_iste(self):
        """Gereken mesajları yüksek hızda iste.

        Duruş açılarını 20 Hz istiyoruz: geolokasyonun en hassas girdisi bu.
        Varsayılan 4 Hz'de, 10 m/s hızla uçarken iki örnek arası 2.5 m eder.
        """
        for msg_id, hz in [
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
            (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 4),
            (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1),
        ]:
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
            time.sleep(0.02)

    def _oku_dongusu(self):
        while not self._dur:
            try:
                m = self.mav.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                if self._dur:
                    return          # kapanış sırasında soket hatası normaldir
                self.log(f"[ucus] okuma hatasi: {e}")
                continue
            if m is None:
                continue
            t = time.time()
            tip = m.get_type()
            with self._kilit:
                d = self._d
                if tip == "GLOBAL_POSITION_INT":
                    d.lat = m.lat / 1e7
                    d.lon = m.lon / 1e7
                    d.alt_agl = m.relative_alt / 1000.0
                    d.alt_amsl = m.alt / 1000.0
                    self._son_konum_t = t
                elif tip == "ATTITUDE":
                    d.roll, d.pitch, d.yaw = m.roll, m.pitch, m.yaw
                    self._son_durus_t = t
                elif tip == "HEARTBEAT":
                    d.mod = MOD_ISIM.get(m.custom_mode, str(m.custom_mode))
                    d.armed = bool(m.base_mode &
                                   mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif tip == "MISSION_CURRENT":
                    d.wp_no = m.seq
                elif tip == "VFR_HUD":
                    d.yer_hizi = m.groundspeed
                elif tip == "GPS_RAW_INT":
                    d.uydu = m.satellites_visible
                    d.gps_fix = m.fix_type
                elif tip == "DISTANCE_SENSOR":
                    # yalnızca aşağı bakan sensörü al (ROTATION_PITCH_270 = 25)
                    if m.orientation == 25:
                        d.lidar_m = m.current_distance / 100.0
                elif tip == "SYS_STATUS":
                    d.batarya_v = m.voltage_battery / 1000.0
                elif tip == "COMMAND_ACK":
                    self._ack[m.command] = (m.result, t)

    # ------------------------------------------------------------------
    # Devralma ve bağlantı gözetimi
    # ------------------------------------------------------------------
    def kontrolu_iste(self, mod="GUIDED"):
        """Yazılımın hangi modda kontrol ettiğini kaydeder.

        Bundan sonra mod bizim komutumuz olmadan değişirse devralma sayılır.
        """
        self._beklenen_mod = mod
        self._devralindi = False

    def kontrolu_birak(self):
        self._beklenen_mod = None

    def devralma_kontrol(self):
        """Pilot devraldıysa PilotDevraldi, telemetri kestiyse BaglantiKayip."""
        d = self.durum()
        if d.konum_yasi > self._baglanti_kayip_s and d.durus_yasi > self._baglanti_kayip_s:
            raise BaglantiKayip(
                f"telemetri {d.konum_yasi:.1f} sn'dir gelmiyor")
        beklenen = self._beklenen_mod
        if beklenen and d.mod != beklenen:
            self._devralindi = True
            self._beklenen_mod = None
            raise PilotDevraldi(f"mod {beklenen} bekleniyordu, {d.mod} okundu")
        return d

    @property
    def devralindi(self):
        return self._devralindi

    def durum(self) -> Durum:
        """Anlık, tutarlı durum fotoğrafı (kopya döndürür)."""
        t = time.time()
        with self._kilit:
            d = Durum(**{k: getattr(self._d, k) for k in self._d.__dataclass_fields__})
            d.zaman = t
            d.konum_yasi = t - self._son_konum_t if self._son_konum_t else 999.0
            d.durus_yasi = t - self._son_durus_t if self._son_durus_t else 999.0
        return d

    def hazir_bekle(self, zaman_asimi=60):
        """Konum ve duruş verisi akmaya başlayana kadar bekler."""
        t0 = time.time()
        while time.time() - t0 < zaman_asimi:
            d = self.durum()
            if d.gecerli:
                self.log(f"[ucus] telemetri hazir. mod={d.mod} uydu={d.uydu} "
                         f"fix={d.gps_fix} irtifa={d.alt_agl:.1f}m")
                return True
            time.sleep(0.2)
        raise TimeoutError("Telemetri akmaya başlamadı (GPS fix / bağlantı kontrol et)")

    # ------------------------------------------------------------------
    # Komutlar
    # ------------------------------------------------------------------
    def _komut(self, komut, *params, onay_bekle=True, zaman_asimi=3.0):
        p = list(params) + [0] * (7 - len(params))
        with self._kilit:
            self._ack.pop(komut, None)      # eski onayi temizle
        gonderim = time.time()
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component, komut, 0, *p)
        if not onay_bekle:
            return True
        # Onayi okuma is parcacigindan bekle -- baglantidan DOGRUDAN okuma!
        while time.time() - gonderim < zaman_asimi:
            with self._kilit:
                kayit = self._ack.get(komut)
            if kayit and kayit[1] >= gonderim:
                sonuc = kayit[0]
                ok = sonuc == mavutil.mavlink.MAV_RESULT_ACCEPTED
                if not ok:
                    self.log(f"[ucus] komut {komut} REDDEDILDI (result={sonuc})")
                return ok
            time.sleep(0.01)
        self.log(f"[ucus] komut {komut} icin onay gelmedi")
        return False

    def mod_ayarla(self, ad, zaman_asimi=5.0):
        """Uçuş modunu değiştirir ve gerçekten değiştiğini DOĞRULAR."""
        if ad not in MOD:
            raise ValueError(f"bilinmeyen mod: {ad}")
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MOD[ad])
        t0 = time.time()
        while time.time() - t0 < zaman_asimi:
            if self.durum().mod == ad:
                self.log(f"[ucus] mod -> {ad}")
                # Kendi mod gecisimiz devralma sanilmasin
                if self._beklenen_mod is not None:
                    self._beklenen_mod = ad
                return True
            time.sleep(0.1)
        self.log(f"[ucus] MOD DEGISMEDI: {ad} (su an {self.durum().mod})")
        return False

    def git(self, lat, lon, alt_agl):
        """GUIDED modda verilen GPS noktasına git (irtifa yerden metre).

        SET_POSITION_TARGET_GLOBAL_INT, ArduPilot GUIDED'da konum hedefi
        vermenin standart yoludur. type_mask ile yalnızca konum bitleri
        etkin bırakılır; hız/ivme/yaw'ı ArduPilot kendi yönetir.
        """
        SADECE_KONUM = 0b0000111111111000
        self.mav.mav.set_position_target_global_int_send(
            0,                                  # time_boot_ms (0 = şimdi)
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            SADECE_KONUM,
            int(lat * 1e7), int(lon * 1e7), float(alt_agl),
            0, 0, 0,       # hız
            0, 0, 0,       # ivme
            0, 0)          # yaw, yaw_rate

    def servo(self, kanal, pwm):
        """Pixhawk servo çıkışını doğrudan sür (yük bırakma)."""
        ok = self._komut(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, kanal, pwm)
        self.log(f"[ucus] servo kanal={kanal} pwm={pwm} -> {'OK' if ok else 'HATA'}")
        return ok

    def gorev_wp_ayarla(self, seq):
        """AUTO görevinde hangi waypoint'ten devam edileceğini belirler.

        GUIDED'a çıkıp yük bıraktıktan sonra AUTO'ya dönerken kullanılır.
        (ArduPilot MIS_RESTART=0 iken zaten kaldığı yerden devam eder; bu
        çağrı bunu garantiye alır.)
        """
        self.mav.mav.mission_set_current_send(
            self.mav.target_system, self.mav.target_component, seq)
        self.log(f"[ucus] gorev waypoint -> {seq}")

    def statustext(self, metin):
        """Yer istasyonunun mesaj panelinde ve tlog'da görünecek not.

        Hakem için tespit kanıtı: ne zaman, nerede, hangi hedef bulundu.
        MAVLink STATUSTEXT alanı 50 bayt ile sınırlıdır.
        """
        try:
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                metin.encode("ascii", "replace")[:50])
        except Exception as e:
            self.log(f"[ucus] statustext hatasi: {e}")

    def arm(self, zaman_asimi=10):
        self._komut(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        t0 = time.time()
        while time.time() - t0 < zaman_asimi:
            if self.durum().armed:
                self.log("[ucus] ARMED")
                return True
            time.sleep(0.2)
        return False

    def kalkis(self, irtifa):
        return self._komut(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, irtifa)

    def in_land(self):
        return self.mod_ayarla("LAND")

    # ------------------------------------------------------------------
    # Bekleme yardımcıları
    # ------------------------------------------------------------------
    def varis_bekle(self, lat, lon, alt_agl, tolerans_m=1.0, irtifa_tolerans=0.6,
                    zaman_asimi=45.0, tekrar_gonder=1.0):
        """Hedef noktaya varılana kadar bekler.

        GUIDED hedefi periyodik tekrar gönderilir: ArduPilot'un GUIDED
        zaman aşımı (GUID_TIMEOUT) tetiklenip aracın fren yapmasını önler.

        Dönüş: (varildi_mi, son_mesafe_m)
        """
        from . import geo
        t0 = time.time()
        son_gonderim = 0.0
        mesafe = float("inf")
        while time.time() - t0 < zaman_asimi:
            # Her turda once kontrol: pilot devraldiysa ya da telemetri
            # kestiyse setpoint gondermeyi ANINDA birak.
            self.devralma_kontrol()
            if time.time() - son_gonderim > tekrar_gonder:
                self.git(lat, lon, alt_agl)
                son_gonderim = time.time()
            d = self.durum()
            if d.lat is None:
                time.sleep(0.1)
                continue
            mesafe = geo.mesafe_m(d.lat, d.lon, lat, lon)
            irtifa_farki = abs((d.irtifa or 0) - alt_agl)
            if mesafe <= tolerans_m and irtifa_farki <= irtifa_tolerans:
                return True, mesafe
            time.sleep(0.15)
        return False, mesafe

    def birakmaya_hazir_bekle(self, lat, lon, alt_agl, tolerans_m, maks_hiz,
                              zaman_asimi=25.0, tekrar_gonder=0.5):
        """Bırakma için HEM konum HEM hız şartı sağlanana kadar bekler.

        varis_bekle tek başına yetmez: hedefin `tolerans` yakınına ilk girdiği
        anda döner, ama araç o anda hâlâ hareket hâlindedir ve konum hatası
        toleransın tamamı kadar olabilir. Bu, isabetin alt sınırını toleransa
        çiviler — geolokasyon ne kadar hassas olursa olsun.

        Burada setpoint sürekli gönderilmeye devam eder (ArduPilot hedefe
        yaklaşmayı sürdürür) ve ancak araç hem yeterince yakın hem de yeterince
        durgun olduğunda dönülür.

        Dönüş: (hazir_mi, mesafe_m, hiz_ms)
        """
        from . import geo
        t0 = time.time()
        son_gonderim = 0.0
        mesafe, hiz = float("inf"), 99.0
        while time.time() - t0 < zaman_asimi:
            self.devralma_kontrol()
            if time.time() - son_gonderim > tekrar_gonder:
                self.git(lat, lon, alt_agl)
                son_gonderim = time.time()
            d = self.durum()
            if d.lat is None:
                time.sleep(0.1)
                continue
            mesafe = geo.mesafe_m(d.lat, d.lon, lat, lon)
            hiz = d.yer_hizi
            if mesafe <= tolerans_m and hiz <= maks_hiz:
                return True, mesafe, hiz
            time.sleep(0.1)
        return False, mesafe, hiz

    def duragan_bekle(self, maks_hiz=0.5, zaman_asimi=8.0):
        """Araç duruncaya kadar bekler (yük bırakmadan hemen önce).

        5 m'den bırakılan yükün düşüş süresi ~1 saniye. 1 m/s artık yatay hız,
        yükü 1 metre öteye taşır ve bu doğrudan puan kaybıdır.
        """
        t0 = time.time()
        hiz = 99.0
        while time.time() - t0 < zaman_asimi:
            self.devralma_kontrol()
            hiz = self.durum().yer_hizi
            if hiz <= maks_hiz:
                return True, hiz
            time.sleep(0.2)
        return False, hiz

    def wp_bekle(self, seq, zaman_asimi=600):
        """AUTO görevinde belirtilen waypoint'e ulaşılmasını bekler."""
        t0 = time.time()
        while time.time() - t0 < zaman_asimi:
            if self.durum().wp_no >= seq:
                return True
            time.sleep(0.2)
        return False

    def kapat(self):
        self._dur = True
        try:
            self.mav.close()
        except Exception:
            pass
