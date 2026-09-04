# Hailo-8L (AI HAT+) Entegrasyonu

## Önce açık konuşalım: `.hef` dosyasını ben üretemem

HEF derlemesi **Hailo Dataflow Compiler** ile yapılır. Bu araç:

- yalnızca **x86_64 Linux**'ta çalışır (Raspberry Pi'de değil, Windows'ta değil),
- Hailo Developer Zone hesabı ister,
- Docker imajı olarak dağıtılır.

Yani HEF'i ya hocanızın kullandığı **Ultralytics dışa aktarma** yolundan, ya da
x86 bir Linux makinede DFC kurarak alırsınız. Aşağıda ikisi de var.

**Şu an elinizdeki HEF `safak_yolov8n.onnx`'ten derlendi — yani v1 modeli.**
Yeni domainde recall'u %12,5 olan model o. Final model eğitilince yeniden
derlenmesi gerekiyor.

---

## Uçuş yolu NCNN, Hailo opsiyonel

`--motor hailo` **açıkça verilmedikçe kullanılmaz**. Otomatik seçim NCNN'de
kalır. Sebep: Hailo yolu kamerayı da GStreamer boru hattına devrediyor ve bu,
sahada doğrulanmadan varsayılan yapılamayacak kadar büyük bir davranış
değişikliği.

Hailo'nun gerçek kazancı **CPU'yu boşaltmak**. RPi5 aynı anda geolokasyon,
JPEG kodlama, MAVLink ve arayüz de çalıştırıyor. Ama kazancın büyüklüğünü
ölçmeden bilemeyiz — `hailo_dogrula.py` FPS'i söyleyecek, NCNN'inkiyle
karşılaştırıp karar veririz. Kazanç küçükse riske girmeye değmez.

---

## Yeni HEF nasıl üretilir

### Yol A — Ultralytics (hocanızın kullandığı, en kolay)

Ultralytics platformunda modeli yükleyip **Hailo** hedefine dışa aktarın.
Hocanızın ayarları ekran görüntüsünde şöyleydi:

```
imgsz=640, simplify=true, conf=0.25, iou=0.7, name=hailo8l
```

`name=hailo8l` önemli — AI HAT+ üzerindeki yonga Hailo-**8L**, Hailo-8 değil.
Yanlış hedef için derlenen HEF yüklenmez.

Yeni modeli (`safak_v2.pt` ya da final model) aynı ayarlarla aktarın.

### Yol B — Yerel Dataflow Compiler (x86 Linux gerekir)

```bash
# 1) PC'de ONNX'e aktar
python tools/export_model.py --format onnx --model models/safak_v2.pt

# 2) x86 Linux'ta, Hailo DFC Docker imajı içinde
hailomz compile yolov8n \
    --ckpt safak_v2.onnx \
    --hw-arch hailo8l \
    --calib-path <kalibrasyon_goruntuleri_klasoru/> \
    --classes 2
```

`--calib-path` için **kendi veri setinizden** 100-500 görüntü verin. COCO
görüntüleriyle kalibre edilirse nicemleme (quantization) bizim brandalarımızın
renk dağılımına göre ayarlanmaz ve doğruluk düşer.

`--classes 2` şart: modelimiz 2 sınıflı (0 = kirmizi_hedef, 1 = mavi_hedef).

---

## Üç sessiz tuzak

Üçü de **hata vermez**. Sadece yanlış sonuç üretir.

### 1. Etiketler — bu bizi sıfırlayabilir

`--labels-json` verilmezse Hailo **COCO'nun 80 sınıfını** kullanır ve model
`person` / `bicycle` döndürür. Bu projede ONNX ile bir kez yaşandı.

Bu depoda dosya hazır — `models/safak_etiketler.json`:

```json
{
  "detection_threshold": 0.35,
  "max_boxes": 20,
  "labels": ["unlabeled", "kirmizi_hedef", "mavi_hedef"]
}
```

⚠️ Listedeki `"unlabeled"` dolgusu **kumar**: bazı sürümlerde gerekli,
bazılarında fazla. Yanlışsa sınıflar bir kayar → **kırmızı ile mavi yer
değişir** → her iki hedefte yanlış yük → 40 puan.

**Savunmamız:** `detect_hailo.py` etiketi **dizeyle** eşliyor, indeksle değil.
- `"unlabeled"` gelirse → tanınmıyor, log'a basılıyor, **eleniyor**
- Kırmızı kutuya `"mavi_hedef"` etiketi gelirse → `color_verify` o kutuda mavi
  oranını ~0 buluyor, eşikte **eleniyor**

Yani hata "yanlış yük atma" değil, "hiç tespit alamama" biçiminde çıkar.
Fail-safe, fail-dangerous değil.

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
