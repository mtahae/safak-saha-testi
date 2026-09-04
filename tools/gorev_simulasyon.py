"""
ŞAFAK UAV - Görev 2 Geolokasyon Doğruluk Simülasyonu
=====================================================
"Hedefi kaç metre hatayla vuracağız?" sorusunu UÇMADAN ölçer.

Yöntem: Bilinen bir konuma sanal hedef koyulur. İHA'nın tarama rotası boyunca
her karede hedef GERÇEK matematikle görüntüye yansıtılır (geo.gps_piksel),
üzerine gerçekçi gürültüler bindirilir, sonra sistem hedefi normal yoldan geri
çözer (geo.piksel_gps + hedef havuzu). Sonuçta bulunan konum ile gerçek konum
arasındaki hata raporlanır.

Modellenen gürültüler:
    - GPS   : yavaş kayan sistematik bias + kare kare rastgele gürültü
    - Duruş : roll/pitch/yaw ölçüm gürültüsü (EKF3 tipik değerleri)
    - İrtifa: barometre kayması (sistematik) + gürültü
    - Piksel: YOLO kutu merkezinin hedef merkezinden sapması

KRİTİK BULGU (kod bunu sayısal olarak gösterir): GPS'in SİSTEMATİK biası
büyük ölçüde KENDİNİ GÖTÜRÜR. Hedefi ölçerken de, hedefe giderken de aynı GPS
alıcısı kullanılır. Alıcı "2 m doğudayım" diye yanılıyorsa, hedefi de 2 m doğuya
koyar; o koordinata giderken yine aynı 2 m yanıldığı için araç fiziksel olarak
gerçek hedefin üstüne gelir. Bu yüzden RTK GPS şart değildir — asıl belirleyici
DURUŞ hatası ve TESPİT İRTİFASIDIR.

    python tools/gorev_simulasyon.py
    python tools/gorev_simulasyon.py --irtifa 15 --gecis 2
"""
import argparse
import math
import random
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.gorev import geo
from src.gorev.hedef_havuzu import HedefHavuzu

# Gerçekçi gürültü seviyeleri (M8N/Here3 sınıfı GPS + ArduPilot EKF3)
GPS_BIAS_M = 2.0            # sistematik sapma (yavaş kayar) — büyük ölçüde götürür
GPS_GURULTU_M = 0.35        # kare kare rastgele
DURUS_GURULTU_DER = 0.6     # roll/pitch ölçüm gürültüsü
YAW_GURULTU_DER = 1.5       # pusula en zayıf halka
IRTIFA_BIAS_M = 0.8         # barometre kayması
IRTIFA_GURULTU_M = 0.15
PIKSEL_GURULTU_PX = 6.0     # YOLO kutu merkezi sapması


def senaryo(irtifa, gecis_sayisi, hiz, kam, G, Y, tohum=None, gps_bias_goturur=True):
    rng = random.Random(tohum)
    lat0, lon0 = 39.9334, 32.8597
    # Hedef, rotanın 8 m yanında (tam altından geçmiyoruz — gerçekçi)
    h_lat, h_lon = geo.oteleme_uygula(lat0, lon0, 40.0, 8.0)

    # GPS'in sistematik biası — uçuş boyunca sabit kabul edilir
    bias_k = rng.gauss(0, GPS_BIAS_M)
    bias_d = rng.gauss(0, GPS_BIAS_M)

    havuz = HedefHavuzu(kume_yaricap_m=4.0, min_tespit=5, log=lambda *a: None)
    gorulen = 0

    for gecis in range(gecis_sayisi):
        # Her geçişte rota biraz kayar (rüzgâr, waypoint toleransı)
        yan_kayma = rng.gauss(0, 3.0)
        yon = 0.0 if gecis % 2 == 0 else math.pi   # ileri-geri geçişler
        for adim in range(0, 90):
            mesafe = adim * hiz * 0.1              # 10 Hz kare
            if gecis % 2 == 0:
                k, d = mesafe, yan_kayma
            else:
                k, d = 90 * hiz * 0.1 - mesafe, yan_kayma
            g_lat, g_lon = geo.oteleme_uygula(lat0, lon0, k, d)

            # --- GERÇEK durum (simülasyonun bildiği) ---
            roll_g = math.radians(rng.gauss(0, 2.0))
            pitch_g = math.radians(rng.gauss(-3.0, 1.5))   # ileri uçuşta burun aşağı
            yaw_g = yon + math.radians(rng.gauss(0, 1.0))

            # Hedef bu karede görünüyor mu? (gerçek matematikle yansıt)
            p = geo.gps_piksel(h_lat, h_lon, G, Y, g_lat, g_lon, irtifa,
                               roll_g, pitch_g, yaw_g, kam)
            if p is None or not p[2]:
                continue
            u, v, _ = p
            u += rng.gauss(0, PIKSEL_GURULTU_PX)
            v += rng.gauss(0, PIKSEL_GURULTU_PX)
            if not (0 <= u < G and 0 <= v < Y):
                continue
            gorulen += 1

            # --- İHA'nın ÖLÇTÜĞÜ (gürültülü) durum ---
            o_lat, o_lon = geo.oteleme_uygula(
                g_lat, g_lon,
                bias_k + rng.gauss(0, GPS_GURULTU_M),
                bias_d + rng.gauss(0, GPS_GURULTU_M))
            o_irtifa = irtifa + IRTIFA_BIAS_M + rng.gauss(0, IRTIFA_GURULTU_M)
            o_roll = roll_g + math.radians(rng.gauss(0, DURUS_GURULTU_DER))
            o_pitch = pitch_g + math.radians(rng.gauss(0, DURUS_GURULTU_DER))
            o_yaw = yaw_g + math.radians(rng.gauss(0, YAW_GURULTU_DER))

            r = geo.piksel_gps(u, v, G, Y, o_lat, o_lon, o_irtifa,
                               o_roll, o_pitch, o_yaw, kam)
            if r is None:
                continue
            havuz.ekle("kirmizi_hedef", r[0], r[1], 0.9, 0.8, o_irtifa)

    k = havuz.en_iyi("kirmizi_hedef")
    if k is None:
        return None, gorulen, 0

    b_lat, b_lon = k.konum
    if gps_bias_goturur:
        # Araç o koordinata giderken AYNI biasla yanıldığı için bias götürür:
        # fiziksel varış noktası = bulunan koordinat - bias
        b_lat, b_lon = geo.oteleme_uygula(b_lat, b_lon, -bias_k, -bias_d)
    return geo.mesafe_m(b_lat, b_lon, h_lat, h_lon), gorulen, k.sayi


def kosu(ad, irtifa, gecis, hiz, kam, G, Y, n=200, **kw):
    hatalar, gorulenler, bulunamadi = [], [], 0
    for i in range(n):
        h, g, n_olcum = senaryo(irtifa, gecis, hiz, kam, G, Y, tohum=i, **kw)
        gorulenler.append(g)
        if h is None:
            bulunamadi += 1
        else:
            hatalar.append(h)
    if not hatalar:
        print(f"{ad:38s} HEDEF HIC BULUNAMADI")
        return
    hatalar.sort()
    print(f"{ad:38s} ort={statistics.mean(hatalar):5.2f}m  "
          f"medyan={statistics.median(hatalar):5.2f}m  "
          f"%95={hatalar[int(.95*len(hatalar))]:5.2f}m  "
          f"en_kotu={hatalar[-1]:5.2f}m  "
          f"kare={statistics.mean(gorulenler):5.1f}  "
          f"kayip={bulunamadi}/{n}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--irtifa", type=float, default=None)
    ap.add_argument("--gecis", type=int, default=None)
    ap.add_argument("--hiz", type=float, default=6.0)
    ap.add_argument("--tekrar", type=int, default=200)
    args = ap.parse_args()

    from src.gorev import gorev_config as cfg
    G, Y = cfg.KAMERA_GENISLIK, cfg.KAMERA_YUKSEKLIK
    kam = geo.KameraModeli(hfov_derece=cfg.KAMERA_HFOV_DERECE)

    print("=" * 96)
    print("SAFAK UAV - GOREV 2 GEOLOKASYON DOGRULUK SIMULASYONU")
    print(f"kamera {G}x{Y} HFOV={cfg.KAMERA_HFOV_DERECE}deg | "
          f"hiz={args.hiz} m/s | {args.tekrar} tekrar/senaryo")
    print(f"gurultu: GPS bias {GPS_BIAS_M}m + {GPS_GURULTU_M}m | "
          f"durus {DURUS_GURULTU_DER}deg | yaw {YAW_GURULTU_DER}deg | "
          f"irtifa {IRTIFA_BIAS_M}m | piksel {PIKSEL_GURULTU_PX}px")
    print("=" * 96)

    if args.irtifa and args.gecis:
        kosu(f"irtifa={args.irtifa}m gecis={args.gecis}", args.irtifa,
             args.gecis, args.hiz, kam, G, Y, args.tekrar)
        return

    print("\n--- TARAMA IRTIFASININ ETKISI (tek gecis) ---")
    for irt in (40, 30, 20, 15, 10):
        kosu(f"irtifa {irt:2d} m, 1 gecis", irt, 1, args.hiz, kam, G, Y, args.tekrar)

    print("\n--- GECIS SAYISININ ETKISI (20 m irtifa) ---")
    for g in (1, 2, 3):
        kosu(f"irtifa 20 m, {g} gecis", 20, g, args.hiz, kam, G, Y, args.tekrar)

    print("\n--- GPS BIASI GERCEKTEN GOTURUYOR MU? (20 m, 2 gecis) ---")
    kosu("bias goturur (gercek durum)", 20, 2, args.hiz, kam, G, Y,
         args.tekrar, gps_bias_goturur=True)
    kosu("bias goturmez (yanlis varsayim)", 20, 2, args.hiz, kam, G, Y,
         args.tekrar, gps_bias_goturur=False)

    print("\nNOT: Sartname siniri 10 m; bu sinirin disi 0 puan.")
    print("Puan d_min/d_tk oranindan geldigi icin hedef 'gecmek' degil, EN KUCUK hata.")


if __name__ == "__main__":
    main()
