"""
ŞAFAK UAV - Bırakma Ofseti Hesaplayıcı
=======================================
Gerçek atışlardan sonra `BIRAKMA_OFSET_KUZEY_M` / `BIRAKMA_OFSET_DOGU_M`
değerlerini hesaplar.

NEDEN GEREKLİ: Yazılım aracı hedefin tam üstüne getirir, ama yük oradan
düşerken bırakma mekanizmasının ittiği yön, rüzgâr, servo gecikmesi ve yükün
yerde yuvarlanması onu sistematik olarak bir yöne kaydırabilir. Şartname
mesafeyi "yükün DURDUĞU yer"den ölçüyor, düştüğü yerden değil.

İŞARET TUZAĞI: Yük hedefin 1.2 m KUZEYİNE düşüyorsa, nişan almamız gereken
yer 1.2 m GÜNEY'dir — yani ofset NEGATİF kuzeydir. Ters yazılırsa hata
iki katına çıkar. Bu araç işareti sizin yerinize hallediyor.

İSTATİSTİK DÜRÜSTLÜĞÜ: Tek atıştan sistematik sapma çıkarılamaz — o atıştaki
rastgele hatayı sistematik sanıp düzeltirseniz işleri kötüleştirirsiniz.
Araç en az 2 ölçüm ister ve saçılma ortalamadan büyükse "bu rastgele, düzeltme
uygulamayın" der.

    python tools/birakma_ofset.py
"""
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.gorev import gorev_config as cfg


def son_rapor():
    """En son uçuş raporunu bulur (bilgi amaçlı gösterilir)."""
    raporlar = sorted(cfg.KANIT_DIZIN.glob("rapor_*.txt"))
    return raporlar[-1] if raporlar else None


def olcum_al(no):
    """Bir atışın ölçümünü alır. (kuzey_m, dogu_m) döndürür ya da None."""
    print(f"\n--- {no}. olcum ---")
    print("  Hedefin MERKEZINDEN, yukun DURDUGU noktaya olan sapma.")
    print("  Giris bicimi:")
    print("    1) mesafe + pusula yonu   ornek: 1.4 45     (1.4 m, kuzeydogu)")
    print("    2) kuzey,dogu metre       ornek: 1.0,1.0")
    print("    (bos birak = bitir)")
    ham = input("  > ").strip()
    if not ham:
        return None
    try:
        if "," in ham:
            k, d = (float(x) for x in ham.split(","))
            return k, d
        parcalar = ham.split()
        if len(parcalar) != 2:
            raise ValueError
        mesafe, yon = float(parcalar[0]), float(parcalar[1])
        # Pusula yonu: 0=Kuzey, 90=Dogu (saat yonunde)
        r = math.radians(yon)
        return mesafe * math.cos(r), mesafe * math.sin(r)
    except ValueError:
        print("  !! anlasilmadi, tekrar deneyin")
        return olcum_al(no)


def main():
    print("=" * 70)
    print("SAFAK UAV - BIRAKMA OFSETI HESAPLAYICI")
    print("=" * 70)

    r = son_rapor()
    if r:
        print(f"\nSon ucus raporu: {r.name}")
        for satir in r.read_text(encoding="utf-8").splitlines():
            if "yatay hata" in satir or "<-" in satir:
                print(f"  {satir.strip()}")
        print("\n  (Yukaridaki 'yatay hata', yazilimin KENDI tahminine gore")
        print("   birakma anindaki sapmasidir. Asagida soracagim ise yukun")
        print("   YERDE nereye dustugudur -- ikisi farkli seylerdir.)")

    print(f"\nMevcut ofset: kuzey {cfg.BIRAKMA_OFSET_KUZEY_M:+.2f} m, "
          f"dogu {cfg.BIRAKMA_OFSET_DOGU_M:+.2f} m")
    print("\nHer atis icin, hedef merkezinden yukun durdugu noktaya olan")
    print("sapmayi girin. En az 2 olcum gerekli.")

    olcumler = []
    no = 1
    while True:
        o = olcum_al(no)
        if o is None:
            break
        k, d = o
        print(f"  kaydedildi: kuzey {k:+.2f} m, dogu {d:+.2f} m "
              f"(mesafe {math.hypot(k,d):.2f} m)")
        olcumler.append((k, d))
        no += 1

    if len(olcumler) < 2:
        print(f"\n{len(olcumler)} olcum girildi. En az 2 gerekli.")
        print("Tek atistan sistematik sapma cikarilamaz: o atistaki RASTGELE")
        print("hatayi sistematik sanip duzeltirseniz isleri kotulestirirsiniz.")
        return 1

    kuzeyler = [o[0] for o in olcumler]
    dogular = [o[1] for o in olcumler]
    ort_k = statistics.mean(kuzeyler)
    ort_d = statistics.mean(dogular)
    ort_mesafe = math.hypot(ort_k, ort_d)
    # Sacilma: her olcumun ortalamadan uzakligi
    sacilma = statistics.mean(math.hypot(k - ort_k, d - ort_d)
                              for k, d in olcumler)

    print("\n" + "=" * 70)
    print(f"{len(olcumler)} olcum")
    print(f"  ortalama sapma : kuzey {ort_k:+.2f} m, dogu {ort_d:+.2f} m "
          f"(buyukluk {ort_mesafe:.2f} m)")
    print(f"  sacilma        : {sacilma:.2f} m")
    print(f"  atislar        : " +
          ", ".join(f"({k:+.1f},{d:+.1f})" for k, d in olcumler))

    if sacilma >= ort_mesafe:
        print("\nSONUC: DUZELTME UYGULAMAYIN.")
        print(f"  Sacilma ({sacilma:.2f} m) ortalama sapmadan ({ort_mesafe:.2f} m)")
        print("  buyuk. Bu, sapmanin sistematik degil RASTGELE oldugu anlamina")
        print("  gelir. Ofset girmek isabeti iyilestirmez, kotulestirebilir.")
        print("  Daha fazla atis yapip tekrar olcun.")
        return 2

    if ort_mesafe < 0.3:
        print("\nSONUC: duzeltmeye gerek yok.")
        print(f"  Ortalama sapma {ort_mesafe:.2f} m, olcum belirsizliginin icinde.")
        return 0

    # İŞARET: yük kuzeye kaçıyorsa güneye nişan al
    ofset_k = -ort_k
    ofset_d = -ort_d

    print("\nSONUC: sistematik sapma var, duzeltilebilir.")
    print(f"\n  Yuk sistematik olarak {ort_mesafe:.2f} m "
          f"{_yon_adi(ort_k, ort_d)} yone kaciyor.")
    print(f"  Bu yuzden {_yon_adi(ofset_k, ofset_d)} yone nisan alacagiz.\n")
    print("  >>> src/gorev/gorev_config.py icine yazin:")
    print(f"      BIRAKMA_OFSET_KUZEY_M = {ofset_k:+.2f}")
    print(f"      BIRAKMA_OFSET_DOGU_M  = {ofset_d:+.2f}")
    print(f"\n  Beklenen iyilesme: ortalama hata {ort_mesafe:.2f} m -> "
          f"~{sacilma:.2f} m (geriye sadece rastgele kisim kalir)")
    print("\n  Duzeltmeden sonra EN AZ BIR atis daha yapip dogrulayin:")
    print("  isaret ters girilirse hata iki katina cikar.")
    return 0


def _yon_adi(k, d):
    if abs(k) < 0.1 and abs(d) < 0.1:
        return "merkez"
    aci = math.degrees(math.atan2(d, k)) % 360
    yonler = ["kuzey", "kuzeydogu", "dogu", "guneydogu",
              "guney", "guneybati", "bati", "kuzeybati"]
    return yonler[int((aci + 22.5) // 45) % 8]


if __name__ == "__main__":
    raise SystemExit(main())
