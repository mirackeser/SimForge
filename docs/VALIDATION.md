# Güncel doğrulama — fizik ve görsel stüdyo

17 Kasım 2026; Linux x86_64, GCC 14.2, CMake 4.4.5.

| Kontrol | Gözlenen sonuç | Kapsam / sınır |
|---|---|---|
| Release CTest | 37/37 geçti | 35 GoogleTest + 2 CLI kontrolü |
| Debug ASan + UBSan | 37/37 geçti | Yerel ptrace ortamı nedeniyle leak taraması kapalı; CI sanitizer işi bu ayarı kullanmaz |
| Sahne koşumları | Studio dahil 7 sahne × 600 tick, çıkış 0 | NaN/Inf kontrolü; her sahnenin fiziksel doğruluk kanıtı değildir |
| Gerçek OpenGL render | Mesa yazılım bağlamında shader/FBO/ImGui/sahne çalıştı; `GL error=0` | 1600×900, 45 tick; aynı Application/Renderer kodu; GPU sürücülerinin tamamını kapsamaz |
| Önizleme | `docs/preview.png` gerçek capture'dan üretildi | Yapay referans görsel kaldırıldı; offline capture FPS göstermez |
| Biçim / derleyici | clang-format kontrolü ve proje uyarı bayraklarıyla derleme geçti | Vendor kaynakları biçim değişikliğinin dışında |

## Fizik referansları

- Serbest düşme: 120 Hz, 1 saniye; analitik hız için 0.0001 m/s, semi-implicit konum için 0.042 m tolerans. Konum hatası sıfır değildir.
- İki eş kütleli küre: yerçekimi/damping/sürtünme kapalı, restitution 1; başlangıç hızları +2 ve -1 m/s. Son hızlar -1 ve +2, momentum ve kinetik enerji 1e-5 toleransla korunuyor.
- Duran kutu: 2 saniyede yükseklik 0.005 m içinde, yapay spin 0.01 rad/s altında.
- Üç kutu istifi: **20 velocity iterasyonu**, 3 saniye; yükseklik toleransları 0.03/0.04/0.06 m. Bu kısa referans, uzun Jenga istifinin her koşulda kararlı olduğunu göstermez.
- Üstel damping: 60 ve 120 Hz'de 1 saniye sonu hızları 1e-4 toleransla eşleşiyor.
- Kutunun büyük zeminle temasında yüz noktaları gerçek örtüşme bölgesinde kalıyor.

## Sınır testleri

Geçersiz broadphase bounds/index, eski entity handle, sıfır/negatif collider boyutu, içerden başlayan veya sıfır yönlü ray, NaN joint ve solver kapasitesini aşan manifold testleri var. Bunlar bağımsız güvenlik sertifikasyonu değildir; ayrıntılı sözleşmeler [QUALITY.md](QUALITY.md) içinde.

## Tekrar üretme

```bash
cmake -S . -B build/headless -DSIMFORGE_BUILD_SANDBOX=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/headless --parallel 2
ctest --test-dir build/headless --output-on-failure --no-tests=error
./build/headless/SimForgeHeadless --scene studio --steps 600 --csv build/studio.csv
# Masaüstü derlemesi, OpenGL bağlamı gerekir:
./build/SimForgePreview build/preview.ppm
```

GitHub CI ayrıca Linux'ta Xvfb altında gerçek `SimForgePreview` çalıştırır ve görüntüyü artifact olarak saklar. Platform sonuçları [Actions](https://github.com/mirackeser/SimForge/actions) sayfasındadır. Gerçek kullanıcı etkileşimi, Retina girdi eşlemesi ve donanım sürücüleri için cihaz üstünde manuel kontrol hâlâ gerekir.

---

## Tarihsel ilk paket doğrulaması

Tarih: 17 Kasım 2026. Yerel ortam: Linux x86_64, GCC 14.2, CMake 4.4.5, C++17.

## Kontroller

| Kontrol | Sonuç | Sınır |
|---|---|---|
| Ekransız Release configure/build | Başarılı | GLM/GoogleTest önceden indirilmiş kaynakları kullanıldı |
| Release CTest | 25/25 geçti | 23 GoogleTest + 2 CLI kontrolü; tüm fizik için kapsam garantisi değildir |
| Debug ASan + UBSan CTest | 25/25 geçti | Yerel çalıştırıcı ptrace kullandığı için `ASAN_OPTIONS=detect_leaks=0` gerekliydi; yerelde leak taraması yapılmadı. CI işi bu kısıtlamayı kapatmaz. |
| Altı demo sahnesi | Her biri 120 tick, çıkış 0 | Sonlu durum kontrolü; fizik doğruluğu/istif stabilitesi kanıtı değildir |
| Aynı cradle deneyinin iki CSV izi | 210 veri satırı, byte düzeyinde eşit | Aynı binary/ayar, 30 tick; CPU/derleyiciler arası garanti değildir |
| Masaüstü hedefinin yerel derlemesi | Başarılı | Yerel sistemde grafik geliştirme paketleri yok; GLFW null backend ile compile/link doğrulandı. Gerçek pencere/UI çalışması denenmedi. |
| Diff boşluk kontrolü | Başarılı | Depodaki mevcut CRLF satır sonları dikkate alındı |

Tekrarlanan cradle CSV SHA-256: `c99836a4b7bb77eb9a3c54711ff79100495c05e56bb6c724b4dfe471c1311296`.

GitHub üzerinde gerçek platform build/test sonuçları [Actions](https://github.com/mirackeser/SimForge/actions) sayfasından izlenir. Yerel masaüstü derlemesi Linux X11/macOS/Windows CI doğrulamasının yerine geçmez.

## Statik zemin optimizasyonu için ilk ölçüm

Aynı çalışma alanında Release runner, her sahne için 120 tick (yaklaşık 1 saniye simülasyon), CSV kapalı. Önce: box bounding sphere hacmiyle grid yerleştirmesi. Sonra: döndürülmüş dünya AABB'si. Diğer bu paket düzeltmeleri iki çalıştırmada da mevcuttu.

| Sahne | Önce duvar saati (s) | Sonra duvar saati (s) |
|---|---:|---:|
| Domino | 8.40 | 0.080 |
| Cradle | 7.46 | 0.061 |
| Jenga | 7.16 | 0.114 |
| Bridge | 7.71 | 0.061 |
| Rain | 6.69 | 0.079 |
| Stress | 7.44 | 0.086 |

Bu tek örnekli, paylaşımlı makinede alınmış bir darboğaz kontrolüdür. Arka plan derleme yükü iki koşumda farklı olabilir; oranlar genel performans garantisi veya yayınlanabilir benchmark değildir. Sağlam benchmark için tekrarlı koşum, medyan/p95, compiler/CPU metadata ve örnek sayıları M3'te eklenecek.

## Tekrar üretme

```bash
cmake -S . -B build/headless -DSIMFORGE_BUILD_SANDBOX=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/headless --parallel 2
ctest --test-dir build/headless --output-on-failure --no-tests=error
./build/headless/SimForgeHeadless --scene cradle --steps 30 --csv build/a.csv
./build/headless/SimForgeHeadless --scene cradle --steps 30 --csv build/b.csv
cmp build/a.csv build/b.csv
```

Sanitizer (GCC/Clang):

```bash
cmake -S . -B build/sanitized -DSIMFORGE_BUILD_SANDBOX=OFF -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build build/sanitized --parallel 2
ctest --test-dir build/sanitized --output-on-failure
```

## Kalan manuel kontroller

Gerçek masaüstünde açılış/kapanış; Retina ve standart DPI'da nesne seçimi; drag'i UI üzerinde bırakma; Space tuşunu basılı tutunca tek geçiş; pause/step/reset; pencere küçültme; UI'nin swap öncesinde görünmesi. Bu kontroller çalıştırılmadan görsel doğrulama tamamlandı sayılmaz.
