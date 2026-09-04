# ŞAFAK UAV — Saha Testi Yapılandırması

Teknofest 2026 Liseler Arası İHA / Döner Kanat **İkinci Görev** yazılımının,
eksik donanımla saha denemesi yapmak için hazırlanmış hâli.

**Bu sürümde iki bayrak açık:**

| Bayrak | Ne yapar |
|---|---|
| `--sadece-mavi` | Yalnızca **2×2 mavi** hedef aranır. Kırmızı hedef ölçülür ve arayüzde görünür ama **üstüne gidilmez** — kırmızı branda yokken ya da zemin kırmızıyken yanlış pozitif peşinde koşmayalım diye |
| `--temsili-servo` | Servo komutu **gönderilmez**. Bunun dışındaki her şey — hizalanma, alçalma, hız şartı, kayıt, göreve dönüş — gerçekte olduğu gibi çalışır |

Bir de `--ui` var: tarayıcıdan canlı izleme arayüzü.

> ⚠️ Bu iki bayrak **varsayılan olarak kapalı**. Yarışma uçuşunda hiçbiri
> verilmez. Hangisi açıksa log'a ve arayüze büyük harfle basılır ki
> yanlışlıkla açık unutulmuş bir bayrakla uçuşa çıkılmasın.

---

## 0. Önce şunu bil

Bu yazılım hedefi **takip etmiyor**. Hedefi gördüğü an GPS koordinatını
hesaplayıp hafızaya yazıyor, sonra o koordinata gidiyor. Hedef kadrajdan
çıksa, gölge geçse, hatta kamera bozulsa bile bırakma tamamlanır.

Piksel merkezleme (görsel servolama) yok — bu yüzden ayarlanacak bir PD
kontrolcü, salınım riski ve "kare kaçtı, her şey sıfırlandı" sorunu da yok.
Merkezleme işini ArduPilot'un kendi konum kontrolcüsü yapıyor.

---

## 1. Raspberry Pi hazırlığı

### 1.1 Soğutma — atlanamaz

```bash
vcgencmd measure_temp
vcgencmd get_throttled
```

`throttled=0x0` olmalı. **Aktif soğutucu takılı değilse devam etme** — bu
proje bir kez termal çökmeyle SD kart kaybetti.

### 1.2 Sistem paketleri

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-venv git
```

`python3-opencv` apt'ten gelmeli; pip'ten kurmaya çalışma, ARM'de saatler sürer.

### 1.3 Kamera görünüyor mu

```bash
rpicam-hello --list-cameras
rpicam-hello -t 5000
```

İlk komut `imx219` yazmalı. Yazmıyorsa şerit kabloyu kontrol et — yazılıma
geçmenin anlamı yok.

### 1.4 Pixhawk bağlantısı

**USB ile** (en kolayı, ilk test için bunu kullan):

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
sudo usermod -aG dialout $USER      # sonra bir kez çıkış-giriş gerekir
```

`/dev/ttyACM0` görüyorsan yolun bu, baud **115200**.

**UART ile** (TELEM2, uçuşta daha güvenilir):

```bash
sudo raspi-config
# Interface Options -> Serial Port
#   "login shell over serial?"      -> NO
#   "serial port hardware enabled?" -> YES
sudo reboot
```
Sonra `/dev/serial0`, baud **921600**.

---

## 2. Kurulum

```bash
cd ~/Desktop
git clone https://github.com/mtahae/safak-saha-testi.git
cd safak-saha-testi
```

### Sanal ortam

**Zaten Hailo venv'in varsa** (`venv_hailo_rpi_examples`) onu kullan:

```bash
source ~/Desktop/hailoenvtest/hailo-rpi5-examples/setup_env.sh
pip install ncnn pymavlink flask
```

**Yoksa yeni kur:**

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements-rpi.txt
```

> **`--system-site-packages` şart.** `picamera2` ve `opencv` apt'ten sistem
> geneline kuruldu; venv onları ancak bu bayrakla görür. `torch` /
> `ultralytics` **kurma**, Pi'de gerekmiyor — model NCNN olarak çalışıyor.

---

## 3. Donanımsız doğrulama (dron kapalıyken)

```bash
python src/gorev/geo.py
```
→ **"TUM TESTLER GECTI"** demeli. Geolokasyon matematiğinin 9 analitik testi.

```bash
python tools/config_denetle.py
```
→ Şu an iki uyarı vermesi normal: HFOV ölçülmemiş ve 1280×720 IMX219'da
kırpık bir mod. İkisini de **Adım 5**'te kapatacağız.

İstersen tüm görevi simüle et (Pixhawk'a gerek yok, sahte MAVLink):

```bash
python tools/gorev_prova.py --gurultu --sadece-mavi --temsili-servo --ui
```
Bu koşarken tarayıcıdan `http://<raspi-ip>:5000` — arayüzü uçmadan görürsün.

---

## 4. Bağlantıyı ayarla

`src/gorev/gorev_config.py` içinde, USB kullanıyorsan:

```python
BAGLANTI = "/dev/ttyACM0"
BAUD = 115200
```

---

## 5. Görüş açısını ÖLÇ — en kritik adım

**Bunu atlarsan geri kalan her şey anlamsız.** Kamera IMX219 ve bu sensörde
çözünürlük seçmek görüş açısını da değiştiriyor:

| Çözünürlük | FOV | Gerçek yatay açı | 20 m'de şerit |
|---|---|---|---|
| 3280×2464 · 1640×1232 · 1640×922 | TAM | 62,2° | 24,1 m |
| **1920×1080** | **KIRPIK** | **38,9°** | 14,1 m |
| **1280×720** | **KIRPIK** | **50,4°** | 18,8 m |
| 640×480 | KIRPIK | 26,5° | 9,4 m |

Yanlış değer hata vermez — **bütün mesafeler aynı oranda şişer** ve yük yanlış
yere düşer.

> Satıcının verdiği "77,6 derece" **köşegen**, yatay değil. Yatay ≈ 62-64°.

```bash
python tools/kamera_kalibrasyon.py --kaynak picam --hfov
```

Kamerayı yerden bilinen bir yüksekliğe (örn. tam 2,00 m) tut, yerde görünen
genişliği mezurayla ölç, araca gir. Çıkan sayıyı `KAMERA_HFOV_DERECE`'ye yaz.

- **62° civarı çıktıysa** → boru hattı kırpmıyor, iyi
- **50° civarı çıktıysa** → 1280×720 kırpık modu kullanılıyor. Ya ölçtüğün
  değeri gir, ya da tam FOV için `KAMERA_GENISLIK/YUKSEKLIK`'i `1640×1232`
  yapıp yeniden ölç (şerit 18,8 m yerine 24,1 m olur, hedefi kaçırma ihtimalin
  düşer)

Sonra montaj yönü:

```bash
python tools/kamera_kalibrasyon.py --kaynak picam --montaj
```
→ Görüntünün üst kenarı dronun neresine bakıyor? `KAMERA_MONTAJ_YAW_DERECE`
(0 = burun, 90 = sağ, 180 = kuyruk, 270 = sol).

---

## 5b. Hailo ile çalışacaksan — doğrulama (ZORUNLU)

HEF depoda hazır: `models/safak_v2.hef` (safak_v2 modelinden, 640×640, NMS
gömülü, hailo8l). Etiket dosyası derleme çıktısıyla doğrulandı.

```bash
sudo apt install hailo-all      # kurulu değilse
hailortcli fw-control identify  # AI HAT+ görünüyor mu
python tools/hailo_dogrula.py
```

Dört şeyi ölçer: boru hattı kare üretiyor mu, etiketler doğru mu, **kırpma
veya letterbox var mı**, FPS ne. Ayrıntı ve gerekçeler: [HAILO.md](HAILO.md).

**Geçmeden `--motor hailo` ile uçma.** Geçemezse NCNN ile uç — görev mantığı
aynı, sadece çıkarım CPU'da olur.

---

## 6. Tezgâh testi — **PERVANELER SÖKÜK**

```bash
python tools/tezgah_testi.py --baglanti /dev/ttyACM0 --baud 115200 --kaynak picam
```

Sekiz adım: bağlantı, GPS, **duruş işareti**, kamera, tespit, servo, mod, yayın.

**En kritiği 3. test.** Dronu elinle öne eğ, sağa yatır — ekrandaki roll/pitch
işaretleri gerçekle uyuşuyor mu? Burada bir işaret tersse geolokasyon hedefi
ayna simetriğine koyar ve bunu ancak sahada, yükü yanlış yere atınca anlarsın.

Tek test koşmak için: `--test 3`

> Servo testini (6) yük mekanizması takılı değilse atla: `--test 1,2,3,4,5,7,8`

---

## 7. Kuru tarama — hâlâ yerde

```bash
python -m src.gorev.gorev2 --kaynak picam --sadece-mavi --temsili-servo --ui --prova
```

Hailo ile (doğrulama geçtiyse — `--kaynak` verilmez, kamerayı boru hattı açar):
```bash
python -m src.gorev.gorev2 --motor hailo --sadece-mavi --temsili-servo --ui --prova
```

`--prova` uçuş komutu da göndermez. Mavi brandayı kameranın önünde gezdir.

Beklenen:
- Log'da `HEDEF DOGRULANDI` ve bir GPS koordinatı
- `runs/gorev2/` altına kutulu kanıt kareleri
- Arayüzde hedef havuzunda `mavi_hedef` satırı, ölçüm sayısı artarken

---

## 8. Canlı uçuş

```bash
# NCNN (CPU) ile
python -m src.gorev.gorev2 --kaynak picam --sadece-mavi --temsili-servo --ui

# Hailo NPU ile
python -m src.gorev.gorev2 --motor hailo --sadece-mavi --temsili-servo --ui
```

`--prova` **yok** — yani araç gerçekten GUIDED'a geçip hedefin üstüne inecek.
Servo yine tetiklenmeyecek.

### Uçuştan önce mutlaka

| Kontrol | Neden |
|---|---|
| **Rotanın sonunda `NAV_LAND` var mı** | Yazılım iniş komutu göndermiyor; yoksa araç son waypoint'te asılı kalır |
| `TESPIT_ACILIS_WP` doğru mu | Bu waypoint'e ulaşınca tespit açılır. Test rotanda hangi numara olduğunu Mission Planner'dan bak |
| `MIS_RESTART = 0` | Yoksa AUTO'ya dönerken görev baştan başlar |
| `SR2_EXTRA1 ≥ 20` | Duruş verisi yavaş gelirse geolokasyon bayat veriyle çalışır |
| Pilot kumandada, elinde | Mod değiştirdiği an yazılım **anında** susar ve kendiliğinden geri almaz |

### Uçuş sırasında arayüzden izle

`http://<raspi-ip>:5000`

- **Faz** — `WP N BEKLENIYOR` → `TARAMA` → `BIRAKMA: mavi_hedef` → `TARAMA`
- **Hedef havuzu** — ölçüm sayısı 5'e ulaşınca `ONAYLI` olur
- **Dağılım** — 3 m'yi aşarsa hedef reddedilir (ölçümler tutarsız demektir)
- **Bırakmalar** — büyük mavi pankart çıkar, `hata` = servo anındaki gerçek
  yatay mesafe. **Puanın geldiği sayı bu.**
- **Algılama sayaçları** — hangi sebeple kaç kare elendiği

---

## Beklenen davranış

```
1. Kumandadan arm + AUTO           -> yazılım hiçbir komut göndermez, izler
2. AUTO rotayı uçar                -> tespit KAPALI (şartname: 2. direkten sonra)
3. TESPIT_ACILIS_WP'ye ulaşıldı    -> tespit AÇILDI (arayüzde yeşil)
4. Mavi branda görüldü             -> her karede GPS'e çevrilip havuza yazılır
5. 5. ölçümde                      -> HEDEF DOGRULANDI, kanıt karesi diske
6. GUIDED'a geçer                  -> 15 m'de hedefin üstüne
7. 5 m'ye alçalır                  -> 2,5 sn durulur
8. Son 8 sn'nin ölçümüyle düzeltir -> 0,4 m + 0,5 m/s şartını bekler
9. TEMSILI SERVO                   -> komut gönderilmez, bırakıldı sayılır
10. AUTO'ya döner                  -> saklanan waypoint'ten devam
11. Rota biter                     -> NAV_LAND ile otonom iner
```

---

## Sorun çıkarsa

| Belirti | Bak |
|---|---|
| Hiç tespit yok | Arayüzde "Algılama sayaçları" → hangi eleme sayacı artıyor? `irtifa` artıyorsa çok alçaksın (min 3 m), `renk` artıyorsa HSV eşiği, `egim` artıyorsa sert manevra |
| Tespit var, `ONAYLI` olmuyor | 5 ölçüm gerekiyor (`MIN_TESPIT_SAYISI`). Hedefin üstünden daha yavaş geç |
| `ONAYLI` oldu ama gitmiyor | Log'a bak: dağılım > 3 m ise reddedilmiştir, ya da hedef 150 m'den uzak hesaplanmıştır (duruş/irtifa verisi bozuk) |
| Yanlış yere gidiyor | HFOV ölçülmemiş (Adım 5) ya da duruş işareti ters (Adım 6, test 3) |
| Arayüz açılmıyor | `pip install flask`; Pi'nin IP'sini `hostname -I` ile al |
| Görev başlamıyor | Arm + **AUTO** ikisi birden gerekli. Arayüzde "Mod" ne yazıyor? |

Uçuş sonrası her şey `runs/gorev2/` altında: log, kanıt kareleri ve
koordinatlı rapor.

---

## Hailo (AI HAT+) kullanacaksan

HEF depoda hazır (`models/safak_v2.hef`). Hailo asla **otomatik** seçilmiyor;
`--motor hailo` ile açıkça istenir ve o modda kamerayı da boru hattı açar
(`--kaynak` verilmez). Önce `tools/hailo_dogrula.py`'yi geçir.
Ayrıntı: [HAILO.md](HAILO.md).

---

## Dosya haritası

| Yol | Ne |
|---|---|
| `src/gorev/gorev_config.py` | **Sahada değişen tek dosya.** Her değerin yanında neden o değer olduğu yazıyor |
| `src/gorev/geo.py` | Piksel ↔ GPS matematiği. `python src/gorev/geo.py` ile öz-test |
| `src/gorev/ucus.py` | ArduPilot/pymavlink katmanı, pilot devralma tespiti |
| `src/gorev/algilayici.py` | Kamera + model + geolokasyon iş parçacığı |
| `src/gorev/hedef_havuzu.py` | Çok kareli kümeleme, medyan konum |
| `src/gorev/gorev2.py` | Durum makinesi. Giriş noktası |
| `test_ui/sunucu.py` | Bu testin izleme arayüzü. Göreve **hiç dokunmaz**, yalnızca okur |
| `tools/` | Kalibrasyon, tezgâh testi, donanımsız prova |
