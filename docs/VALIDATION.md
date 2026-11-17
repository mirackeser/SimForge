# İlk paket doğrulaması

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
