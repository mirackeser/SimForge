# Kod, fizik ve güvenlik sözleşmeleri

## Kod yapısı

Proje C++17 kullanır. Çekirdek, scene, desktop ve test hedefleri ayrıdır. Temas geometri üretimi `Collision`, impuls iterasyonları `ContactSolver`, zaman/dünya akışı `PhysicsWorld`, yüzey shader'ları `Shaders.hpp` içindedir. Birim ve davranış testleri gerçek kütüphanelere bağlanır. `.clang-format` ve `.editorconfig` biçimi tanımlar; proje hedefleri GCC/Clang'da `-Wall -Wextra -Wpedantic`, MSVC'de `/W4` ile derlenir. Vendor kodu bu biçim düzenlemesinin dışındadır.

Kaynak kodda LLM ağı, gizli API anahtarı, shell komutu yürütme veya kullanıcı kodu çalıştırma yolu yoktur. Hatalar CLI sınırında açıklanır; grafik kaynakları exception durumunda da bağlam kapanmadan bırakılır. Kullanılmayan birleşik contact overload'u ve etkisiz joint damping alanı kaldırılmıştır.

## Girdi ve kaynak sınırları

| Alan | Davranış |
|---|---|
| Sayısal durum | NaN/Inf transform, velocity, inertia, kütle ve malzeme değerleri reddedilir; quaternion birim olmalıdır |
| Collider | Pozitif ve sonlu radius/half-extents; sıfıra yakın/sayısal aralık dışı inertia reddedilir |
| Solver | Timestep 0.00001–0.1 s, cell size 0.05–100, velocity iterasyon 1–64, position iterasyon 0–32 |
| Entity | `createEntity` en çok 10.000 kayıt oluşturur; kimlik taşması reddedilir; clear eski kimliği yeniden kullanmaz |
| Broadphase | Collider başına 262.144 hücre, toplam 2.000.000 giriş ve 2.000.000 çift ziyareti bütçesi; koordinat dönüşümünden önce int aralığı kontrol edilir |
| Dizi erişimi | Query uzunlukları ve hücre indeksleri erişimden önce doğrulanır; manifold sayısı sekiz nokta ile sınırlıdır |
| Joint | Anchor, hedef mesafe, stiffness ve damping alanları fizik adımından önce doğrulanır |
| Ray | Sonlu birim yön gerekir; içerden başlayan ray pozitif çıkış yüzeyini ve sonlu normali döndürür |
| Grafik | Shader/FBO başarısızlığı hata üretir; geçici shader/program temizlenir; kapanış idempotenttir |
| Debug | 64 küre × 72 iz noktası; 1.024 grid hücresi ve 100.000 çizgi segmenti görünüm bütçesi |

Bu sınırlar interaktif masaüstü sandbox'ı içindir. Aşırı veya geçersiz programatik girdiler exception üretir. `entitiesMutable()` düşük seviye API'si, vektör büyütüldüğünde component pointer'larını geçersiz kılar; pointer'lar entity ekleme/silme arasında saklanmamalıdır. Dışarıdan sahne dosyası henüz okunmaz; eklendiğinde dosya sınırında ayrı şema/boyut doğrulaması gerekir.

## Bağımlılıklar

GLM, GLFW, ImGui ve GoogleTest kaynakları değişmez commit SHA'sına sabitlidir. CMake ilk configure sırasında ağdan kaynak indirir; uygulama çalışırken bunları indirmez. GitHub işlerinin repository izni `contents: read` seviyesindedir. SHA sabitlemek sürüm takibi ve güvenlik güncellemelerinin yerine geçmez.

## Doğrulamanın anlamı

ASan/UBSan testleri ve sınır kontrolleri incelenen davranışlar için bellek/sayısal hata yakalamayı amaçlar; bütün olası açıkların yokluğunu kanıtlamaz. Bu çalışma bağımsız penetrasyon testi veya tüm bağımlılıklar için CVE sertifikasyonu değildir. Fizik, görsel ve güvenlik sonuçları çalıştırılan testlerle birlikte raporlanır.
