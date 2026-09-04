# Hailo-8L (AI HAT+) Entegrasyonu

## HEF hazır — depoda

`models/safak_v2.hef` (9,4 MB). **safak_v2 modelinden** derlenmiş.
Yanındaki iki dosya derleme ayarlarının kanıtı:

`models/safak_v2_hailo_metadata.yaml`:
```yaml
names:  {0: kirmizi_hedef, 1: mavi_hedef}
imgsz:  [640, 640]
nms:    true
hailo_arch: hailo8l
end2end: false
```

`models/safak_v2_nms_config.json`:
```json
{ "classes": 2, "background_removal": false,
  "nms_scores_th": 0.25, "nms_iou_th": 0.7, "image_dims": [640, 640] }
```

Bunlar iki soruyu kesin olarak cevaplıyor:

**1. Etiket sırası.** `background_removal: false` → arka plan sınıfı **yok**,
indeks 0 doğrudan `kirmizi_hedef`. Bu yüzden `models/safak_etiketler.json`
başına `"unlabeled"` dolgusu **konulmaz**:

```json
{ "detection_threshold": 0.25, "max_boxes": 20,
  "labels": ["kirmizi_hedef", "mavi_hedef"] }
```

Dolgu konulsaydı sınıflar bir kayardı: kırmızı `"unlabeled"` (tanınmaz, elenir),
mavi `"kirmizi_hedef"` (renk doğrulaması eler, çünkü mavi kutuda kırmızı yok).
**Sonuç sıfır tespit olurdu — hata vermeden.** Bu tuzağı tahminle değil derleme
çıktısıyla kapattık.

**2. NMS.** `nms: true` → bastırma HEF'in içinde. `detect_hailo.py` zaten
NMS'lenmiş tespit bekliyor, uyumlu.

## `.hef` yeniden nasıl üretilir

Yeni bir model eğitildiğinde gerekir. HEF derlemesi **Hailo Dataflow
Compiler** ile yapılır: yalnızca x86_64 Linux'ta çalışır, Hailo hesabı ister,
Docker imajı olarak dağıtılır — Raspberry Pi'de veya Windows'ta üretilemez.

**Yol A — Ultralytics (bu HEF böyle üretildi):** modeli platforma yükleyip
Hailo hedefine aktarın. Kullanılan ayarlar: `imgsz=640, simplify=true,
conf=0.25, iou=0.7, name=hailo8l`. `hailo8l` önemli — AI HAT+'taki yonga
Hailo-8**L**.

**Yol B — Yerel DFC (x86 Linux):**
```bash
python tools/export_model.py --format onnx --model models/safak_v2.pt
hailomz compile yolov8n --ckpt safak_v2.onnx --hw-arch hailo8l         --classes 2 --calib-path <kendi_goruntulerimiz/>
```
Kalibrasyon görüntüleri **kendi veri setimizden** olmalı; COCO'yla kalibre
edilirse nicemleme brandalarımızın renk dağılımına göre ayarlanmaz.

## Uçuş yolu NCNN, Hailo opsiyonel

`--motor hailo` **açıkça verilmedikçe kullanılmaz**. Otomatik seçim NCNN'de
kalır. Sebep: Hailo yolu kamerayı da GStreamer boru hattına devrediyor ve bu,
sahada doğrulanmadan varsayılan yapılamayacak kadar büyük bir davranış
değişikliği.

Hailo'nun gerçek kazancı **CPU'yu boşaltmak**. RPi5 aynı anda geolokasyon,
JPEG kodlama, MAVLink ve arayüz de çalıştırıyor. Ama kazancın büyüklüğünü
ölçmeden bilemeyiz — `hailo_dogrula.py` FPS'i söyleyecek, NCNN'inkiyle
karşılaştırıp karar veririz. Kazanç küçükse riske girmeye değmez.

## Üç sessiz tuzak

Üçü de **hata vermez**. Sadece yanlış sonuç üretir.

### 1. Etiketler — ÇÖZÜLDÜ

`--labels-json` verilmezse Hailo **COCO'nun 80 sınıfını** kullanır ve model
`person` / `bicycle` döndürür. Bu projede ONNX ile bir kez yaşandı.

Dosya hazır ve sırası **derleme çıktısıyla doğrulandı** (yukarıya bakınız):
`["kirmizi_hedef", "mavi_hedef"]`, başta dolgu yok.

**Yine de savunma katmanı duruyor:** `detect_hailo.py` etiketi *dizeyle*
eşliyor, indeksle değil. Beklenmedik bir etiket gelirse log'a basılıp eleniyor;
sınıf kayması olsa bile `color_verify` yakalıyor (kırmızı kutuda mavi oranı ≈ 0).
Yani hata "yanlış yük atma" değil, "hiç tespit alamama" biçiminde çıkar.

### 2. Renk düzeni

`get_numpy_from_buffer` **RGB** döndürür, `color_verify` **BGR** bekler.
Çevrilmezse HSV'de kırmızı ile mavi yer değişir. `detect_hailo.py` callback'te
çeviriyor.

### 3. Kırpma veya germe → geolokasyon sapar

GStreamer boru hattı kareyi modelin girişine hazırlarken iki şey yapabilir:

- **Kırpma** → gerçek görüş açısı daralır → `KAMERA_HFOV_DERECE` yanlış kalır
- **Anamorfik germe** (en/boy oranını korumadan kareye germe) → dikey ölçek
  yataydan farklı olur → tek odak uzaklığı kullanmak hedefi yanlış yere koyar

İkincisini ölçtük: 16:9 kare 640×640'a gerildiğinde, merkezin 160 piksel
üstündeki bir hedef için yatay mesafe **6,49 m yerine 3,68 m** çıkıyor — %76
hata.

Bunun için `KameraModeli.vfov_derece` var (`None` = kare piksel, varsayılan).
`hailo_dogrula.py` kırpma ile germeyi ayırt edip girilecek değeri hesaplıyor.

---

## Doğrulama — göreve almadan önce ZORUNLU

```bash
sudo apt install hailo-all       # kurulu değilse
python tools/hailo_dogrula.py
```

Dört şeyi tek seferde ölçer:

1. **Boru hattı** — arka planda çalışıyor mu, kare üretiyor mu
2. **Etiketler** — `kirmizi_hedef` / `mavi_hedef` mi geliyor, yoksa COCO mu
   (COCO gelirse tek tek listeler)
3. **Kırpma / germe** — boru hattının karesini picamera2'nin tam-FOV karesiyle
   yan yana kaydeder, en/boy oranını karşılaştırır, gerekirse VFOV değerini
   hesaplar
4. **Hız** — FPS. NCNN'e göre kazanç var mı?

Çıktılar `runs/hailo_dogrula/` altına düşer. **Geçmeden `--motor hailo` ile
uçmayın.**

---

## Bilinen risk

`detect_hailo.py` ve `hailo_dogrula.py` **donanımda hiç çalıştırılmadı.** Söz
dizimi doğrulandı, mevcut testler geçiyor, ama Hailo'ya özgü satırlar ilk kez
sahada çalışacak.

Bilinen bir risk: `app.run()` bir GLib main loop başlatıyor ve GLib bazen ana
iş parçacığı bekliyor. Arka plan iş parçacığında çalışmazsa `hailo_dogrula.py`
20 saniyede zaman aşımı verip bunu söyleyecek — sessizce ölmeyecek.

---

## Mimari: neden `detect_hailo.py` hem kamera hem dedektör

Hailo'nun Python arayüzü bir GStreamer boru hattı ve kamerayı **kendi açıyor**:

| | Bizim mimari | Hailo |
|---|---|---|
| Model | **Çekme** (`kam.oku()` → `ded.isle()`) | **İtme** (callback) |
| Kamera sahibi | `Kamera` sınıfımız | GStreamer boru hattı |
| Ana döngü | Bizim iş parçacığımız | `app.run()` — **bloklar** |

`HailoKaynak` ikisini bağlıyor: boru hattını arka planda çalıştırıp her karede
**(kare + tespit) ÇİFTİNİ** kilitli olarak saklıyor, dışarıya hem `Kamera` hem
`Dedektör` arayüzü sunuyor.

Çift olarak saklamak şart: `Algilayici` önce `oku()` sonra `isle()` çağırıyor.
Arada boru hattı yeni kare üretirse, kare N'in görüntüsüyle kare N+1'in tespiti
eşleşir ve geolokasyon **sessizce** yanlış koordinat üretir.

Bu sayede `algilayici.py`, `gorev2.py`, `hedef_havuzu.py`, `ucus.py` ve
`yayin.py`'ye hiç dokunulmadı.

```bash
# Hailo ile kuru tarama (doğrulama geçtikten SONRA)
python -m src.gorev.gorev2 --motor hailo --sadece-mavi --temsili-servo --ui --prova
```
