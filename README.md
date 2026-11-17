# SimForge — 3D Physics Sandbox

C++17 ile yazılmış küre/OBB rijit cisim simülasyonu, OpenGL masaüstü sandbox'ı ve ekransız deney çalıştırıcısı.

![SimForge Preview](docs/preview.png)

Mevcut özellikler: quaternion açısal entegrasyon, eylemsizlik tensörleri, sphere/sphere ve sphere/OBB çarpışmaları, 15 eksenli OBB SAT, Coulomb sürtünmesi, mesafe/yay kısıtları, spatial hash broadphase, mouse spring ile seçim ve ImGui profiler. OBB temas çözümü tek nokta yaklaşımıdır; üretim motoru doğruluğu veya platformlar arasında bit düzeyinde determinism iddiası yoktur.

**AI durumu:** Henüz model/ajan entegrasyonu yoktur. SimForge AI deney asistanına giden geliştirme aşamaları [yol haritasında](docs/ROADMAP.md) tanımlıdır.

## Proje belgeleri

- [Geliştirme planı, öncelikler ve kabul koşulları](docs/ROADMAP.md)
- [Katmanlar, CMake hedefleri ve fizik tick sözleşmesi](docs/ARCHITECTURE.md)
- [Kod incelemesi, hata bulguları ve ilk paket](docs/AUDIT.md)
- [Doğrulama sonuçları, ölçümler ve sınırlar](docs/VALIDATION.md)

## Derleme

CMake 3.20+, C++17 derleyicisi ve Git gerekir. CMake GLM 1.0.1, GLFW 3.4, ImGui 1.91.5 ve GoogleTest 1.15.2 bağımlılıklarını ilk kurulumda indirir. GLAD depoda bulunur.

### macOS

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
./build/SimForge
ctest --test-dir build --output-on-failure
```

### Windows — Visual Studio 2022

```powershell
cmake -S . -B build
cmake --build build --config Release --parallel 2
.\build\Release\SimForge.exe
ctest --test-dir build -C Release --output-on-failure
```

### Linux — Debian/Ubuntu

```bash
sudo apt-get install cmake g++ git libx11-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev libgl1-mesa-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
./build/SimForge
ctest --test-dir build --output-on-failure
```

Linux masaüstü varsayılan olarak X11 kullanır. Wayland geliştirme bağımlılıkları kuruluysa `-DSIMFORGE_ENABLE_WAYLAND=ON` ile Wayland desteği de açılır.

### Ekransız derleme ve deney

Pencere sistemi veya OpenGL geliştirme paketleri gerektirmez:

```bash
cmake -S . -B build/headless -DSIMFORGE_BUILD_SANDBOX=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/headless --parallel 2
ctest --test-dir build/headless --output-on-failure
./build/headless/SimForgeHeadless --scene cradle --steps 120 --csv trace.csv
```

Windows çoklu konfigürasyon derlemesinde çalıştırıcı `build\headless\Release\SimForgeHeadless.exe` altındadır. Sahne anahtarları: `domino`, `cradle`, `jenga`, `bridge`, `rain`, `stress`. `--help` seçenekleri gösterir. Adım sayısı 1–1.000.000 aralığındadır. Her adım tam bir sabit tick'tir (varsayılan 1/120 saniye). NaN/Inf fizik durumu veya dosya hatası sıfır olmayan çıkış üretir.

CSV her aktif cismin tick sonu konum, hız, quaternion ve açısal hızını içerir. Aynı binary ve girdilerle fizik izleri karşılaştırılabilir; CSV tam bir sahne/replay dosyası değildir. Büyük sahnelerde çıktı boyutu ve çalışma süresi adım sayısıyla artar.

| CMake seçeneği | Varsayılan | Amaç |
|---|---|---|
| `SIMFORGE_BUILD_SANDBOX` | `ON` | Grafik masaüstü uygulaması |
| `SIMFORGE_BUILD_HEADLESS` | `ON` | CSV üreten ekransız çalıştırıcı |
| `SIMFORGE_BUILD_TESTS` | `ON` | Birim/entegrasyon testleri |
| `SIMFORGE_ENABLE_WAYLAND` | `OFF` | Linux GLFW Wayland backend'i |

## Kontroller

| Girdi | İşlem |
|---|---|
| Sol fare sürükleme | Cismi mouse spring ile tut/fırlat |
| Sağ fare sürükleme | Kamerayı döndür |
| `+` / `-` | Yakınlaş/uzaklaş |
| `Space` | Duraklat/devam et |
| `Step Frame` | Duraklat ve tam bir fizik tick'i çalıştır |
| ImGui paneli | Sahne seç, sıfırla, cisim ekle, solver/sürtünme/dönüş ayarlarını değiştir |
| `Esc` | Çıkış |

## Örnek sahneler

Domino zinciri, Newton beşiği, Jenga kulesi, asılı köprü, 180 küreli partikül yağmuru ve 36 kutu + 80 küreli karma stres sahnesi. Bunlar etkileşimli gösterimlerdir; analitik doğrulama benchmark'larının yerine geçmez.

## Doğrulama

GoogleTest testleri çarpışma, inertia, quaternion entegrasyonu, raycast, mesafe kısıtı, çift sırası ve fizik dünyasının zaman/reset/yay/ayar davranışlarını kapsar. CTest ayrıca headless CLI'nin başarılı çalışma ve geçersiz girdi yollarını kontrol eder.

GitHub Actions masaüstünü Linux/macOS/Windows üzerinde derler ve testleri çalıştırır; ek Linux işinde ekransız çekirdek AddressSanitizer/UndefinedBehaviorSanitizer ile sınanır. UI etkileşimleri için manuel masaüstü kontrolü de gerekir.

## Lisans

[MIT](LICENSE).
