"""
ŞAFAK UAV - Görev 2 Yapılandırması
===================================
Görev katmanının TÜM ayarlanabilir parametreleri burada. Sahada değiştirilecek
tek dosya budur; kodun içine sabit değer gömülmez.

Görev akışı (şartname 10.2.2, Liseler Arası Döner Kanat İkinci Görev):
    Kalkış -> AUTO görev (verilen waypoint'ler) -> direk 2 dıştan alınır
    -> TESPİT AÇILIR -> hedefler GPS'e çevrilir -> AUTO duraklatılır
    -> GUIDED ile hedeflerin üstüne gidilip yükler bırakılır
    -> AUTO'ya dönülür -> bitiş çizgisi -> otonom iniş

Hedef/yük eşlemesi (ÇAPRAZ - şartname):
    2x2 m MAVİ hedef   <- dışı KIRMIZI yük
    1x1 m KIRMIZI hedef <- dışı MAVİ yük
"""
from pathlib import Path

PROJE_KOK = Path(__file__).resolve().parent.parent.parent

# ===========================================================================
# 1) UÇUŞ KONTROLCÜSÜ BAĞLANTISI
# ===========================================================================
# RPi5 -> Cube Orange+ TELEM2 seri bağlantısı (varsayılan).
# SITL testi için: "udpin:127.0.0.1:14551" veya "tcp:127.0.0.1:5763"
BAGLANTI = "/dev/serial0"
BAUD = 921600

# Bu yazılımın MAVLink kimliği (yer istasyonu 255, FC 1'dir; çakışmasın)
KENDI_SYSTEM_ID = 191
KENDI_COMPONENT_ID = 191

# ===========================================================================
# 2) KAMERA
# ===========================================================================
# KULLANILAN KAMERA: Sony IMX219 (Raspberry Pi Camera Module v2 muadili).
# Camera Module 3 (IMX708) kırıldığı için bununla değiştirildi -- ikisinin
# görüş açısı FARKLIDIR, eski 66° değeri IMX708'e aitti ve artık geçersizdir.
#
# --- TUZAK 1: satıcının verdiği açı KÖŞEGEN ---
# Satıcı sayfası "Görüş Alanı: 77.6 derece" diyor. Bu YATAY değil KÖŞEGEN.
# Ölçüldü (3280x2464 aktif alan, 1.12 um piksel, f=2.96 mm):
#     yatay 63.6°   dikey 50.0°   köşegen 75.6°
# Resmi Raspberry Pi Camera v2 değerleri: yatay 62.2°, dikey 48.8°.
# Köşegeni yatay sanmak tüm mesafeleri ~%25 şişirir.
#
# --- TUZAK 2: IMX219'un bazı modları SENSÖRÜ KIRPAR ---
# Bu sensörde çözünürlük seçmek görüş açısını da değiştirir:
#     3280x2464  TAM FOV          yatay 62.2°   20 m'de 24.1 m şerit
#     1640x1232  TAM FOV (binned) yatay 62.2°   20 m'de 24.1 m şerit
#     1640x922   TAM FOV, 16:9    yatay 62.2°   20 m'de 24.1 m şerit
#     1920x1080  KIRPIK           yatay 38.9°   20 m'de 14.1 m şerit
#     1280x720   KIRPIK           yatay 50.4°   20 m'de 18.8 m şerit
#     640x480    KIRPIK           yatay 26.5°   20 m'de  9.4 m şerit
# picamera2/libcamera bu eski kırpma davranışını her zaman uygulamaz (tam
# sensör alanını ölçekleyebilir) -- ama UYGULARSA hata vermez, sadece tüm
# mesafeler ~%38 şişer ve hedef yanlış yere düşer.
#
# BU YÜZDEN: aşağıdaki değer bir BAŞLANGIÇ TAHMİNİDİR (tam FOV varsayımı).
# Uçuştan önce tools/kamera_kalibrasyon.py --hfov ile MUTLAKA ölçün:
# bilinen irtifada yerde görünen genişliği ölçüp W = 2*h*tan(HFOV/2)'den
# geri çözer. Hailo yolu kullanılacaksa tools/hailo_dogrula.py da bakar.
KAMERA_HFOV_DERECE = 62.2

# Dikey görüş açısı. None = KARE PİKSEL (normal kamera yolu için doğru olan bu).
# Yalnızca görüntü boru hattı kareyi ANAMORFIK gerdiğinde doldurulur — yani
# en/boy oranını korumadan ölçeklediğinde. Hailo/GStreamer yolu 16:9 sensör
# karesini 640x640 KAREYE gerebilir; o zaman dikey ölçek yataydan farklı olur
# ve tek odak uzaklığı kullanmak hedefi sistematik olarak yanlış yere koyar.
# Hata vermez, sessizce sapar. tools/hailo_dogrula.py ölçer ve değeri söyler.
KAMERA_VFOV_DERECE = None

# Kamera gövdeye göre nasıl dönük? Optik eksen aşağı (nadir) bakar varsayılır.
# 0   -> görüntünün ÜST kenarı burna bakıyor (en yaygın montaj)
# 90  -> üst kenar sağ kanada bakıyor
# 180 -> üst kenar kuyruğa bakıyor
# 270 -> üst kenar sol kanada bakıyor
KAMERA_MONTAJ_YAW_DERECE = 0.0

# Kameranın nadirden sabit sapması (mekanik montaj hatası, derece).
# Kalibrasyonla ölçülür; 0 bırakmak çoğu durumda yeterlidir.
KAMERA_EGIM_DUZELTME_DERECE = 0.0   # pitch yönünde (+ ileri bakar)
KAMERA_YATIS_DUZELTME_DERECE = 0.0  # roll  yönünde (+ sağa bakar)

# Çıkarım için kameradan okunacak çözünürlük.
# DİKKAT (IMX219): 1280x720 eski libcamera yolunda KIRPILMIŞ bir moddur.
# Kırpma olursa yukarıdaki HFOV yanlış kalır. Tam FOV garantisi isteniyorsa
# 1640x1232 (4:3) ya da 1640x922 (16:9) kullanın -- ikisi de tam sensör
# alanını 2x2 binning ile okur. Ölçüm yapılmadan hangisinin geçerli olduğu
# bilinemez; kamera_kalibrasyon.py karar verdirir.
KAMERA_GENISLIK = 1280
KAMERA_YUKSEKLIK = 720
KAMERA_FPS = 30

# ===========================================================================
# 3) TESPİT KAPISI — şartname: tespit, 2. direk dıştan alındıktan SONRA
# ===========================================================================
# AUTO görevindeki bu waypoint numarasına ULAŞILDIKTAN sonra tespit açılır.
# Sahada waypoint'leri yükledikten sonra Mission Planner'daki gerçek sıra
# numarasını buraya yazın (direk 2'yi dıştan aldıktan hemen sonraki WP).
TESPIT_ACILIS_WP = 4

# Tespit, bu waypoint'e ulaşınca kapanır (tarama alanı geçildi).
# None ise görev sonuna kadar açık kalır.
TESPIT_KAPANIS_WP = None

# ===========================================================================
# 4) HEDEF HAVUZU — çoklu kareden güvenilir konum
# ===========================================================================
# Tek karelik tespite asla güvenilmez. Aynı yere düşen tespitler kümelenir,
# medyanları alınır. Bu hem GPS/attitude gürültüsünü hem de tek karelik
# yanlış pozitifleri temizler.
KUME_YARICAP_M = 4.0        # bu yarıçaptaki tespitler aynı hedef sayılır
MIN_TESPIT_SAYISI = 5       # bir kümenin "hedef" ilan edilmesi için gereken tespit
MIN_GUVEN = 0.45            # görev sırasında YOLO güven eşiği (eğitimdekinden yüksek)
MIN_RENK_ORANI = 0.35       # HSV renk doğrulama alt sınırı
MAKS_YATAY_MESAFE_M = 60.0  # bu kadar uzağa düşen projeksiyonlar atılır (ufuk hatası)
MAKS_EGIM_DERECE = 25.0     # |roll| veya |pitch| bunu aşarsa kare geolokasyona katılmaz
# DİKKAT: Bu değer BIRAKMA_IRTIFA_M'den belirgin KÜÇÜK olmalı. Aksi hâlde
# hedefin tam üstünde, bırakma irtifasında alınan -- yani en isabetli --
# ölçümler elenir ve bırakma öncesi son düzeltme çalışmaz. (Prova testinde
# 5.0/5.0 iken 1033 ölçüm bu yüzden çöpe gitmişti.)
MIN_IRTIFA_M = 3.0          # bu irtifanın altında geolokasyon güvenilmez

# ---------------------------------------------------------------------------
# TARAMA İRTİFASI — ölçümle belirlendi, tahminle değil
# ---------------------------------------------------------------------------
# Test setinde her hedef istenen piksel boyutuna getirilip 1280x720 kadraja
# oturtularak modelin recall'u ölçüldü (NCNN, imgsz=512, conf=0.45):
#
#   hedef px | 1x1 kirmizi irtifa | recall | 2x2 mavi irtifa | recall
#        100 |               10 m |   100% |            20 m |    94%
#         50 |               20 m |   100% |            39 m |    78%
#         40 |               25 m |    95% |            49 m |    56%
#         33 |               30 m |    58% |            60 m |    28%
#         25 |               39 m |     0% |            79 m |     0%
#
# Model ~35 pikselin altında hedefi göremiyor. 1x1 m hedef için bu 28 m
# demek. Bu yüzden AUTO görevindeki tarama waypoint'lerinin irtifası 20 m
# olmalı; 28 m mutlak tavandır.
#
# NOT: Bu yüzden imgsz'i 640'a çıkarmaya GEREK YOK. 20 m'de 512 zaten %100
# recall veriyor; 640 istemek NCNN modelinin yeniden export edilmesini
# gerektirir (model 512 girişle sabit derlenmiş — 640 verilirse hata vermez,
# sessizce SIFIR tespit üretir) ve ~%26 FPS kaybettirir.
TARAMA_IRTIFA_HEDEF_M = 20.0
TESPIT_MAKS_IRTIFA_M = 28.0   # bunun üstünde tespit güvenilmez, uyarı verilir

# ===========================================================================
# 5) BIRAKMA
# ===========================================================================
YAKLASMA_IRTIFA_M = 15.0    # hedefe bu irtifada gidilir
BIRAKMA_IRTIFA_M = 5.0      # bırakma bu irtifada yapılır (alçak = az saçılma)
KONUM_TOLERANS_M = 1.0      # yaklaşma/alçalma fazlarında "vardı" sayılan mesafe

# BIRAKMA anındaki tolerans. Bu değer, isabetin ALT SINIRINI belirler: araç
# hedefin 1 m yakınında "vardım" deyip bırakırsa, geolokasyon 0.3 m hassas olsa
# bile yük 1 m öteye düşer. Bu yüzden son yaklaşma ayrı ve daha sıkı yapılır.
# Çok küçük seçilirse ArduPilot GUIDED bu bandın içine oturamaz ve zaman aşımı
# olur; 0.4 m, konum tutuş hassasiyeti ile ulaşılabilirlik arasında denge.
SON_KONUM_TOLERANS_M = 0.4
SON_YAKLASMA_ZAMAN_ASIMI_S = 25.0
IRTIFA_TOLERANS_M = 0.6
DURULMA_SANIYE = 2.5        # bırakmadan önce salınım sönsün diye beklenen süre
VARIS_ZAMAN_ASIMI_S = 45.0  # bu sürede varılamazsa bırakmayı yine de dene

# Servo kanalları (Pixhawk AUX çıkışları; SERVOn_FUNCTION = 0 "Disabled" olmalı
# ki MAVLink DO_SET_SERVO ile doğrudan sürülebilsin).
SERVO_KANAL_KIRMIZI_YUK = 9    # dışı KIRMIZI yük -> MAVİ (2x2) hedefe
SERVO_KANAL_MAVI_YUK = 10      # dışı MAVİ  yük -> KIRMIZI (1x1) hedefe
SERVO_KILITLI_PWM = 1100
SERVO_ACIK_PWM = 1900
SERVO_ACIK_KALMA_S = 1.5       # pim çekili kalma süresi

# Hedef sınıfı -> hangi yükü bırakacağız (ÇAPRAZ eşleme, şartname)
YUK_ESLEME = {
    "mavi_hedef":    ("kirmizi_yuk", SERVO_KANAL_KIRMIZI_YUK),
    "kirmizi_hedef": ("mavi_yuk",    SERVO_KANAL_MAVI_YUK),
}

# Bırakma önceliği: 1x1 kırmızı hedef 20 puan, 2x2 mavi hedef 10 puan.
# Batarya/süre biterse önce değerli olanı at.
BIRAKMA_SIRASI = ["kirmizi_hedef", "mavi_hedef"]

# Bırakma anında araç bu hızın altına inmiş olmalı. 5 m'den bırakılan yükün
# düşüş süresi ~1 saniye; 1 m/s artık hız, yükü 1 metre öteye taşır. Beklemek
# birkaç saniye, kaçırılan metre ise doğrudan puan kaybı.
MAKS_BIRAKMA_HIZI = 0.5        # m/s
HIZ_BEKLEME_ZAMAN_ASIMI_S = 8.0

# Bırakma ofseti — İLK GERÇEK ATIŞTAN SONRA doldurulur.
# Yükün nereye düştüğünü metreyle ölçün; sistematik bir sapma varsa (rüzgâr,
# mekanizmanın fırlatma yönü, servo gecikmesi) buraya ters işaretiyle yazın.
# Örn. yük sürekli 1.2 m kuzeye düşüyorsa: BIRAKMA_OFSET_KUZEY_M = -1.2
BIRAKMA_OFSET_KUZEY_M = 0.0
BIRAKMA_OFSET_DOGU_M = 0.0

# ===========================================================================
# 5b) GÜVENLİK — arıza ve devralma davranışı
# ===========================================================================
# Pilot kumandadan modu değiştirirse (LOITER/STABILIZE/ALT_HOLD...) yazılım
# DERHAL komut göndermeyi bırakır. Pilotla kavga eden bir companion, aracı
# düşüren en yaygın sebeplerden biridir. Devralmadan sonra yazılım bir daha
# kendiliğinden kontrolü almaz — sadece tespit ve yayına devam eder.
DEVRALMA_KONTROLU = True

# MAVLink'ten bu süre boyunca mesaj gelmezse bağlantı kopmuş sayılır.
BAGLANTI_KAYIP_S = 3.0

# Bir hedef kümesinin ölçüm dağılımı bunu aşarsa güvenilmez sayılır ve
# üstüne UÇULMAZ. Yüksek dağılım = bozuk irtifa/duruş verisi ya da iki farklı
# nesnenin aynı kümede birleşmesi demektir.
MAKS_KUME_DAGILIM_M = 3.0

# Hedef, aracın o anki konumundan bu kadar uzaksa reddedilir. Bozuk bir
# duruş ölçümü hedefi yüzlerce metre öteye koyabilir; bu kontrol olmadan
# araç oraya doğru uçar.
MAKS_HEDEF_MESAFE_M = 150.0

# Batarya bu gerilimin altına inerse yeni bırakma denemesi başlatılmaz
# (süregelen bırakma tamamlanır) ve göreve dönülür. 6S için 21.0 yapın.
MIN_BATARYA_V = 14.0

# ===========================================================================
# 6) SÜRE GÜVENLİĞİ — şartname: 2. görev uçuşu en fazla 10 dakika
# ===========================================================================
GOREV_SURE_LIMIT_S = 10 * 60
# Bu süreye gelindiğinde hedef bulunmuş olsun olmasın bırakma denemeleri
# kesilir ve göreve (bitiş çizgisi + iniş) dönülür.
BIRAKMAYI_BIRAK_S = 8 * 60

# ===========================================================================
# 7) YER İSTASYONUNA KANIT — şartname: tespit hakeme kanıtlanmak ZORUNDA
# ===========================================================================
# Annotated görüntünün UDP ile aktarılacağı yer istasyonu adresi
YAYIN_HEDEF_IP = "192.168.1.50"
YAYIN_PORT = 5600
YAYIN_BITRATE_KBPS = 2500
YAYIN_ACIK = True

# Tespitler MAVLink STATUSTEXT olarak da gönderilir -> Mission Planner mesaj
# panelinde ve tlog'da görünür (hakem için ikinci, bağımsız kanıt).
STATUSTEXT_ACIK = True

# Her tespit anının annotated karesi diske de yazılır (uçuş sonrası kanıt)
KANIT_DIZIN = PROJE_KOK / "runs" / "gorev2"

# ===========================================================================
# 8) MODEL
# ===========================================================================
# RPi5'te NCNN, PC/SITL testinde PyTorch kullanılır. gorev2.py otomatik seçer.
NCNN_MODEL_DIZIN = PROJE_KOK / "models" / "safak_yolov8n_ncnn_model"
PT_MODEL = PROJE_KOK / "models" / "safak_yolov8n.pt"

# --- Hailo-8L NPU (Raspberry Pi AI HAT+) --------------------------------
# İSTEĞE BAĞLI dördüncü motor. `--motor hailo` ile açıkça istenmedikçe
# KULLANILMAZ; otomatik seçim NCNN'de kalır. Sebep: Hailo yolu kamerayı
# GStreamer boru hattına devrediyor ve bu, sahada doğrulanana kadar test
# edilmiş NCNN yolundan daha riskli.
HAILO_HEF = PROJE_KOK / "models" / "safak_v2.hef"
# ETİKET DOSYASI ZORUNLU. Verilmezse Hailo COCO'nun 80 sınıfını kullanır ve
# model "person"/"bicycle" döndürür (bu proje bunu bir kez yaşadı).
#
# Listede "unlabeled" dolgusu YOK ve olmamalı. Derleme çıktısındaki
# nms_config.json şunu söylüyor:
#     "classes": 2,  "background_removal": false
# background_removal false ise arka plan sınıfı YOKTUR; indeks 0 doğrudan
# kirmizi_hedef'tir. Başa "unlabeled" koymak sınıfları bir kaydırır:
# kırmızı "unlabeled" (elenir), mavi "kirmizi_hedef" olur ve renk
# doğrulamasında elenir -> SIFIR TESPİT. Hata vermez, sadece hiçbir şey bulmaz.
HAILO_ETIKET_JSON = PROJE_KOK / "models" / "safak_etiketler.json"
# GStreamer kaynağı: "rpi" (CSI kamera), "usb", /dev/videoN ya da video dosyası
HAILO_KAYNAK = "rpi"
