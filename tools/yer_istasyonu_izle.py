"""
ŞAFAK UAV - Yer İstasyonu Kanıt İzleyici
=========================================
İHA'dan gelen annotated canlı görüntüyü yer istasyonu bilgisayarında gösterir.
Hakeme "hedefi görüntü işlemeyle bulduk" kanıtını sunduğumuz ekran budur.

    python tools/yer_istasyonu_izle.py                 # 5600 portunu dinle
    python tools/yer_istasyonu_izle.py --port 5600 --kaydet

Mission Planner ile YAN YANA açın: solda Mission Planner haritası (İHA'nın
konumu + STATUSTEXT mesajları), sağda bu pencere (tespit kutuları). İkisi
birlikte tespitin hem görüntüsünü hem koordinatını gösterir.

'q' = çık, 'k' = o anki kareyi diske kaydet
"""
import argparse
import socket
import struct
import time
from pathlib import Path

import cv2
import numpy as np

BASLIK = struct.Struct("!IHH")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--kaydet", action="store_true", help="tum akisi mp4 olarak kaydet")
    ap.add_argument("--dizin", default="runs/yer_istasyonu")
    args = ap.parse_args()

    dizin = Path(args.dizin)
    dizin.mkdir(parents=True, exist_ok=True)

    sok = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sok.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 22)
    sok.bind(("0.0.0.0", args.port))
    sok.settimeout(2.0)
    print(f"[yer] {args.port} portu dinleniyor... (q = cik, k = kare kaydet)")

    parcalar = {}          # kare_no -> {parca_no: veri}
    beklenen = {}          # kare_no -> parca_sayisi
    yazici = None
    son_kare_zamani = time.time()
    alinan, dusen = 0, 0

    while True:
        try:
            paket, _ = sok.recvfrom(2048)
        except socket.timeout:
            print(f"[yer] veri yok ({time.time()-son_kare_zamani:.0f}s) — "
                  f"IHA yayinda mi, ayni agda misiniz?")
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        if len(paket) < BASLIK.size:
            continue
        kare_no, parca_no, toplam = BASLIK.unpack(paket[:BASLIK.size])
        parcalar.setdefault(kare_no, {})[parca_no] = paket[BASLIK.size:]
        beklenen[kare_no] = toplam

        if len(parcalar[kare_no]) != toplam:
            continue

        veri = b"".join(parcalar[kare_no][i] for i in range(toplam))
        # Tamamlanan kareden ESKİ, yarım kalmış kareleri temizle
        for k in [k for k in parcalar if k < kare_no]:
            parcalar.pop(k, None)
            beklenen.pop(k, None)
            dusen += 1
        parcalar.pop(kare_no, None)
        beklenen.pop(kare_no, None)

        kare = cv2.imdecode(np.frombuffer(veri, np.uint8), cv2.IMREAD_COLOR)
        if kare is None:
            continue
        alinan += 1
        son_kare_zamani = time.time()

        if args.kaydet:
            if yazici is None:
                h, w = kare.shape[:2]
                yol = dizin / f"kanit_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
                yazici = cv2.VideoWriter(str(yol), cv2.VideoWriter_fourcc(*"mp4v"),
                                         12.0, (w, h))
                print(f"[yer] kaydediliyor: {yol}")
            yazici.write(kare)

        cv2.putText(kare, f"SAFAK UAV - CANLI  kare={alinan} dusen={dusen}",
                    (10, kare.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)
        cv2.imshow("SAFAK UAV - Yer Istasyonu Kaniti", kare)
        t = cv2.waitKey(1) & 0xFF
        if t == ord("q"):
            break
        if t == ord("k"):
            yol = dizin / f"kare_{time.strftime('%H%M%S')}.jpg"
            cv2.imwrite(str(yol), kare)
            print(f"[yer] kaydedildi: {yol}")

    if yazici:
        yazici.release()
    cv2.destroyAllWindows()
    print(f"[yer] bitti. alinan={alinan} dusen={dusen}")


if __name__ == "__main__":
    main()
