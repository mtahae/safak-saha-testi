"""
ŞAFAK UAV - Sahte Pixhawk (MAVLink Uçuş Kontrolcüsü Simülatörü)
================================================================
Gerçek MAVLink konuşan, ArduPilot gibi davranan bir simülatör. Görev
yazılımını (src/gorev/) donanım olmadan uçtan uca koşturmak için.

NEDEN: ArduPilot SITL'i Windows'a kurmak saatler alır ve 6 günlük hazırlıkta
bu zamanı harcamaya değmez. Bu simülatör, görev yazılımının GERÇEKTEN test
edilmesi gereken kısmını test eder: durum makinesi doğru sırayla mı ilerliyor,
AUTO doğru anda mı kesiliyor, hedefe gidiliyor mu, servo doğru kanaldan mı
tetikleniyor, göreve dönülüyor mu.

TCP sunucu olarak açılır — ArduPilot SITL ile AYNI adres biçimi. Yani daha
sonra gerçek SITL kurulduğunda görev yazılımının komut satırı hiç değişmez.

Neyi simüle ETMEZ: aerodinamik, EKF, rüzgâr, gerçek batarya. Bunlar için
gerçek SITL veya saha testi gerekir. Burada amaç MANTIK doğrulaması.
"""
import math
import threading
import time

from pymavlink import mavutil

MOD_NO = {"STABILIZE": 0, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}
MOD_AD = {v: k for k, v in MOD_NO.items()}

R_DUNYA = 6378137.0


def _oteleme(lat, lon, kuzey, dogu):
    dlat = kuzey / R_DUNYA
    dlon = dogu / (R_DUNYA * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def _mesafe(lat1, lon1, lat2, lon2):
    dk = math.radians(lat2 - lat1) * R_DUNYA
    dd = math.radians(lon2 - lon1) * R_DUNYA * math.cos(math.radians(lat1))
    return math.hypot(dk, dd), dk, dd


class SahtePixhawk(threading.Thread):
    """ArduPilot Copter gibi davranan MAVLink düğümü."""

    YATAY_HIZ = 6.0     # m/s
    DIKEY_HIZ = 2.0     # m/s

    def __init__(self, adres="tcpin:127.0.0.1:5763", ev_lat=39.9334, ev_lon=32.8597,
                 gorev=None, log=print, gurultu=False):
        super().__init__(daemon=True)
        self.log = log
        # gurultu=True iken YAYINLANAN telemetri bozulur ama GERCEK (truth)
        # durum temiz kalir. Sahte kamera truth'tan cizdigi icin, gorev
        # yazilimi tipki sahadaki gibi "yanlis" telemetriyle dogru goruntuyu
        # eslestirmek zorunda kalir. Gercek entegrasyon testi budur.
        self.gurultu = gurultu
        import random as _r
        self._rng = _r.Random(42)
        self._bias_k = self._rng.gauss(0, 2.0) if gurultu else 0.0
        self._bias_d = self._rng.gauss(0, 2.0) if gurultu else 0.0
        self._bias_alt = self._rng.gauss(0, 0.8) if gurultu else 0.0
        self.adres = adres
        self.ev_lat, self.ev_lon = ev_lat, ev_lon

        # --- gerçek (truth) durum ---
        self.lat, self.lon = ev_lat, ev_lon
        self.alt = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.mod = "STABILIZE"
        self.armed = False
        self.wp_no = 0
        self.hiz = 0.0

        self.gorev = gorev or []          # [(kuzey_m, dogu_m, irtifa, tip), ...]
        self.guided_hedef = None          # (lat, lon, alt)
        self.servo_kayit = []             # [{zaman, kanal, pwm, lat, lon, alt}]
        self.son_setpoint_t = 0.0         # en son GUIDED setpoint'inin geldigi an
        self.pilot_devraldi_t = None      # senaryonun pilot devralmasini tetikledigi an
        self.statustext_kayit = []
        self._dur = False
        self._kilit = threading.Lock()
        self.baglandi = False

    # ------------------------------------------------------------------
    def run(self):
        self.mav = mavutil.mavlink_connection(self.adres, source_system=1,
                                              source_component=1)
        self.log(f"[sahte-fc] dinleniyor: {self.adres}")
        # tcpin bağlantı kabul edene kadar bloklar
        self.mav.wait_heartbeat(timeout=60)
        self.baglandi = True
        self.log("[sahte-fc] gorev yazilimi baglandi")

        t_hb = t_pos = t_att = t_diger = 0.0
        t_son = time.time()
        while not self._dur:
            simdi = time.time()
            dt = simdi - t_son
            t_son = simdi

            self._gelen_mesajlari_isle()
            self._fizik(dt)

            if simdi - t_hb > 1.0:
                self._heartbeat(); t_hb = simdi
            if simdi - t_pos > 0.1:
                self._konum(); t_pos = simdi
            if simdi - t_att > 0.05:
                self._durus(); t_att = simdi
            if simdi - t_diger > 0.5:
                self._diger(); t_diger = simdi
            time.sleep(0.005)

    # ------------------------------------------------------------------
    def _gelen_mesajlari_isle(self):
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "COMMAND_LONG":
                self._komut(m)
            elif t == "SET_MODE":
                self._mod_ayarla(MOD_AD.get(m.custom_mode, "?"))
            elif t == "SET_POSITION_TARGET_GLOBAL_INT":
                with self._kilit:
                    self.guided_hedef = (m.lat_int / 1e7, m.lon_int / 1e7, m.alt)
                    # Devralma testinin dogrulayacagi sey: pilot kontrolu
                    # aldiktan SONRA bu zaman damgasi ilerlemeyi birakmali.
                    self.son_setpoint_t = time.time()
            elif t == "MISSION_SET_CURRENT":
                with self._kilit:
                    self.wp_no = m.seq
                self.log(f"[sahte-fc] gorev wp -> {m.seq}")
            elif t == "STATUSTEXT":
                metin = m.text if isinstance(m.text, str) else m.text.decode(errors="replace")
                self.statustext_kayit.append(metin)
                self.log(f"[sahte-fc] <<STATUSTEXT>> {metin}")

    def _komut(self, m):
        c = m.command
        onay = mavutil.mavlink.MAV_RESULT_ACCEPTED
        if c == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            kanal, pwm = int(m.param1), int(m.param2)
            # Servo tetiklendigi andaki GERCEK konum kaydedilir. Yukun fiziksel
            # olarak nereye dustugunu belirleyen budur; gorev yazilimin kendi
            # (gurultulu) konum tahmini degil. Puan bu sayidan gelir.
            with self._kilit:
                self.servo_kayit.append({"zaman": time.time(), "kanal": kanal,
                                         "pwm": pwm, "lat": self.lat,
                                         "lon": self.lon, "alt": self.alt})
            self.log(f"[sahte-fc] *** SERVO kanal={kanal} pwm={pwm} "
                     f"@ {self.lat:.7f},{self.lon:.7f} irtifa={self.alt:.1f}m ***")
        elif c == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            with self._kilit:
                self.armed = bool(m.param1)
            self.log(f"[sahte-fc] {'ARMED' if self.armed else 'DISARMED'}")
        elif c == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            pass          # zaten yüksek hızda yayınlıyoruz
        elif c == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            pass
        else:
            onay = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        self.mav.mav.command_ack_send(c, onay)

    def _mod_ayarla(self, ad):
        if ad == "?":
            return
        with self._kilit:
            onceki, self.mod = self.mod, ad
            if ad == "GUIDED":
                # ArduPilot GUIDED'a girerken mevcut konumda tutar
                self.guided_hedef = None
        self.log(f"[sahte-fc] mod {onceki} -> {ad}")

    # ------------------------------------------------------------------
    def _fizik(self, dt):
        if dt <= 0 or dt > 0.5:
            return
        with self._kilit:
            if not self.armed:
                return
            if self.mod == "AUTO":
                hedef = self._auto_hedef()
            elif self.mod == "GUIDED":
                hedef = self.guided_hedef
            elif self.mod == "LAND":
                hedef = (self.lat, self.lon, 0.0)
            else:
                hedef = None
            if hedef is None:
                self.hiz = 0.0
                return

            h_lat, h_lon, h_alt = hedef
            mesafe, dk, dd = _mesafe(self.lat, self.lon, h_lat, h_lon)

            # Yatay hareket
            if mesafe > 0.05:
                adim = min(self.YATAY_HIZ * dt, mesafe)
                self.lat, self.lon = _oteleme(self.lat, self.lon,
                                              dk / mesafe * adim, dd / mesafe * adim)
                self.hiz = adim / dt
                self.yaw = math.atan2(dd, dk)
                # İleri uçuşta burun aşağı, dönüşte hafif yatış — geolokasyonun
                # duruş açılarını gerçekten kullandığını test etmek için önemli
                self.pitch = math.radians(-4.0)
            else:
                self.hiz = 0.0
                self.pitch = 0.0

            # Dikey hareket
            d_alt = h_alt - self.alt
            if abs(d_alt) > 0.05:
                self.alt += math.copysign(min(self.DIKEY_HIZ * dt, abs(d_alt)), d_alt)

            # AUTO'da waypoint'e varılınca sıradakine geç
            if self.mod == "AUTO" and mesafe < 2.0 and abs(d_alt) < 1.0:
                if self.wp_no < len(self.gorev) - 1:
                    self.wp_no += 1
                    self.log(f"[sahte-fc] waypoint {self.wp_no} "
                             f"({self.gorev[self.wp_no][3]})")
                elif self.gorev[self.wp_no][3] == "LAND" and self.alt < 0.2:
                    self.armed = False
                    self.log("[sahte-fc] GOREV BITTI, DISARM")

    def _auto_hedef(self):
        if not self.gorev or self.wp_no >= len(self.gorev):
            return None
        k, d, a, tip = self.gorev[self.wp_no]
        lat, lon = _oteleme(self.ev_lat, self.ev_lon, k, d)
        return lat, lon, (0.0 if tip == "LAND" else a)

    # ------------------------------------------------------------------
    def _heartbeat(self):
        with self._kilit:
            mod, armed = self.mod, self.armed
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
             (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)),
            MOD_NO.get(mod, 0), mavutil.mavlink.MAV_STATE_ACTIVE)

    def _konum(self):
        with self._kilit:
            lat, lon, alt, yaw = self.lat, self.lon, self.alt, self.yaw
        if self.gurultu:
            lat, lon = _oteleme(lat, lon,
                                self._bias_k + self._rng.gauss(0, 0.35),
                                self._bias_d + self._rng.gauss(0, 0.35))
            alt = alt + self._bias_alt + self._rng.gauss(0, 0.15)
        self.mav.mav.global_position_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            int(lat * 1e7), int(lon * 1e7),
            int((alt + 850) * 1000), int(alt * 1000),
            0, 0, 0, int(math.degrees(yaw) * 100) % 36000)

    def _durus(self):
        with self._kilit:
            r, p, y = self.roll, self.pitch, self.yaw
        if self.gurultu:
            g = math.radians
            r += g(self._rng.gauss(0, 0.6))
            p += g(self._rng.gauss(0, 0.6))
            y += g(self._rng.gauss(0, 1.5))
        self.mav.mav.attitude_send(
            int(time.time() * 1000) & 0xFFFFFFFF, r, p, y, 0, 0, 0)

    def _diger(self):
        with self._kilit:
            wp, hiz, alt = self.wp_no, self.hiz, self.alt
        self.mav.mav.mission_current_send(wp)
        self.mav.mav.gps_raw_int_send(
            int(time.time() * 1e6), 3, int(self.lat * 1e7), int(self.lon * 1e7),
            int(alt * 1000), 80, 80, int(hiz * 100), 0, 18)
        self.mav.mav.vfr_hud_send(hiz, hiz, int(math.degrees(self.yaw)) % 360,
                                  50, alt, 0)
        self.mav.mav.sys_status_send(0, 0, 0, 200, 15800, -1, 85, 0, 0, 0, 0, 0, 0)

    def durdur(self):
        self._dur = True
