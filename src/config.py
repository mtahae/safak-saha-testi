"""
ŞAFAK UAV - Hedef Tespiti - Merkezi Yapılandırma
=================================================
Tüm yollar, sınıf tanımları, HSV renk bantları ve eşik değerleri burada toplanır.
Diğer scriptler (auto_label, split_dataset, train, eval, detect) buradan okur.
"""

from pathlib import Path

# ---------------------------------------------------------------------------
# Yollar (proje köküne göre)
# ---------------------------------------------------------------------------
PROJE_KOK = Path(__file__).resolve().parent.parent      # .../safakuav
FOTO_DIZIN = PROJE_KOK / "foto"                          # ham JPG fotoğraflar
DATASET_DIZIN = PROJE_KOK / "dataset"
IMG_DIZIN = DATASET_DIZIN / "images"                    # train/val/test alt klasörleri
LBL_DIZIN = DATASET_DIZIN / "labels"
DATA_YAML = DATASET_DIZIN / "data.yaml"
MODEL_DIZIN = PROJE_KOK / "models"
OTOETIKET_DIZIN = PROJE_KOK / "foto"                     # auto_label çıktısı (.txt) fotoların yanına yazılır

# ---------------------------------------------------------------------------
# Sınıf tanımları  (renge göre — irtifadan bağımsız, sağlam)
#   kırmızı kare = 1x1 m hedef
#   mavi   kare  = 2x2 m hedef
# ---------------------------------------------------------------------------
SINIF_ISIMLERI = ["kirmizi_hedef", "mavi_hedef"]
SINIF_ID = {"kirmizi_hedef": 0, "mavi_hedef": 1}
KIRMIZI = 0
MAVI = 1

# ---------------------------------------------------------------------------
# HSV renk bantları (OpenCV: H 0-179, S 0-255, V 0-255)
# Branda fotoğraflarına göre kalibre edildi; auto_label ve color_verify kullanır.
# Kırmızı, Hue çemberinin iki ucuna sarktığı için iki bant ile aranır.
# ---------------------------------------------------------------------------
HSV_KIRMIZI_ALT_1 = (0, 90, 80)
HSV_KIRMIZI_UST_1 = (12, 255, 255)
HSV_KIRMIZI_ALT_2 = (155, 90, 80)   # 165->155: magenta/pembeye kayan kirmizi brandalari da kapsar (bkz DJI_0585 testi)
HSV_KIRMIZI_UST_2 = (179, 255, 255)

# Branda açık camgöbeği (cyan) mavi -> Hue ~85-110
HSV_MAVI_ALT = (85, 70, 70)
HSV_MAVI_UST = (120, 255, 255)

# ---------------------------------------------------------------------------
# Otomatik etiketleme filtre eşikleri
# ---------------------------------------------------------------------------
# Kontur, kare görüntü alanının en az bu oranı kadar olmalı (çok küçük lekeleri ele)
MIN_ALAN_ORANI = 0.0005      # 4000x2250 -> ~4500 px; uzak/küçük hedefleri de yakalar
MAX_ALAN_ORANI = 0.60        # kareyi neredeyse dolduran dev maskeleri ele
# Kare şekil filtreleri — DÖNDÜRÜLMÜŞ dikdörtgen (minAreaRect) üzerinden hesaplanır
# (eğik açıdan brandalar elmas gibi görünür; rotasyona dayanıklı ölçüm gerekir).
KARE_ORAN_MAX = 2.4          # minAreaRect uzun kenar / kısa kenar (perspektifle artar; şerit/yol >2.4)
MINRECT_DOLULUK_MIN = 0.55   # kontur alanı / minAreaRect alanı (parlama/gölge boşlukları toleransı)
SOLIDITY_MIN = 0.85          # kontur alanı / convex hull alanı (ana yanlış-pozitif filtresi)
# Morfoloji kernel oranları (görüntünün kısa kenarına göre)
ACMA_KERNEL_ORANI = 0.003    # open: küçük lekeleri temizler
KAPAMA_KERNEL_ORANI = 0.012  # close: parlama/kırışık boşluklarını köprüler (parça birleştirir)

# ---------------------------------------------------------------------------
# Tespit / renk doğrulama eşikleri (detect.py, color_verify.py)
# ---------------------------------------------------------------------------
IMGSZ = 960                  # PC/eğitim çıkarım çözünürlüğü (eğitimle aynı)
GUVEN_ESIGI = 0.35           # YOLO minimum güven
IOU_ESIGI = 0.50             # NMS
RENK_DOGRULAMA_MIN_ORAN = 0.30   # bbox içinde beklenen rengin min piksel oranı

# ---------------------------------------------------------------------------
# Dağıtım (RPi5) ayarları — detect_onnx.py / detect_ncnn.py
# ---------------------------------------------------------------------------
# tools/benchmark_imgsz.py ile test bölümünde ölçüldü:
#   960 -> mAP50 0.9937 / R 1.000 | 640 -> 0.9937 / 0.972 | 512 -> 0.9950 / 1.000
#   416 -> 0.9700 / 0.969          | 320 -> 0.9147 / 0.911
# 512, doğruluktan hiç ödün vermeden (en yüksek mAP50, recall=1.0) 640'a göre ~%26
# daha hızlı. 416'nın altında küçük/uzak hedefler kaçmaya başlıyor.
IMGSZ_DAGITIM = 512

# RPi5'in 4 çekirdeğinin TAMAMINI doyurmak ısı ve akım tepesini gereksiz büyütüyor
# (ilk denemede sistem termal korumaya girip kapandı ve SD kart bozuldu).
# Bir çekirdeği boşta bırakmak sistemi ayakta tutar, FPS'i çok az düşürür.
CIKARIM_THREAD = 3
