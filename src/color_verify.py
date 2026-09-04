"""
ŞAFAK UAV - Renk Doğrulama (İP6 yardımcı)
=========================================
YOLO'nun bulduğu bir kutunun, beklenen sınıfın rengini gerçekten taşıyıp taşımadığını
HSV ile doğrular. Aldatıcıları (kırmızı tuğla, mavi bank, vb.) ve sınıf karışıklığını
ikinci bir katman olarak eler.

Ana fonksiyon: renk_orani(bgr, kutu, sinif_id) -> beklenen renk piksel oranı (0-1)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402


def _maske(hsv, alt, ust):
    return cv2.inRange(hsv, np.array(alt, np.uint8), np.array(ust, np.uint8))


def renk_maskesi(bgr, sinif_id):
    """Verilen sınıf için (kırmızı/mavi) HSV renk maskesini döndürür."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if sinif_id == config.KIRMIZI:
        return _maske(hsv, config.HSV_KIRMIZI_ALT_1, config.HSV_KIRMIZI_UST_1) | \
            _maske(hsv, config.HSV_KIRMIZI_ALT_2, config.HSV_KIRMIZI_UST_2)
    return _maske(hsv, config.HSV_MAVI_ALT, config.HSV_MAVI_UST)


def renk_orani(bgr, kutu, sinif_id):
    """
    kutu = (x1, y1, x2, y2) piksel koordinatları.
    Kutu içindeki piksellerin ne kadarının beklenen renkte olduğunu döndürür (0-1).
    """
    x1, y1, x2, y2 = [int(v) for v in kutu]
    h, w = bgr.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    roi = bgr[y1:y2, x1:x2]
    maske = renk_maskesi(roi, sinif_id)
    return float(np.count_nonzero(maske)) / (roi.shape[0] * roi.shape[1])


def dogrula(bgr, kutu, sinif_id, esik=None):
    """Renk oranı eşiğin üstündeyse True (gerçek hedef), değilse False (aldatıcı)."""
    if esik is None:
        esik = config.RENK_DOGRULAMA_MIN_ORAN
    return renk_orani(bgr, kutu, sinif_id) >= esik
