# Başlangıç incelemesi ve ilk geliştirme paketi

17 Kasım 2026; başlangıç commit'i `304ee95`.

## Gözlenen durum

Depoda tek başlangıç commit'i, 9 GoogleTest testi ve 6 örnek fizik sahnesi vardı. AI servisi/modeli, sahne dosyası, deney manifesti ve kullanıcı verisi katmanı bulunmuyordu. `SimForgeTests` tüm fizik döngüsünü içermiyor; kaynakların yalnızca bir bölümünü ayrıca derliyordu.

Mevcut CI run `39026573605`: macOS başarılı, Windows build başarısız, Linux configure başarısız. Windows loglarında `min/max` makrolarının GLM/`std::max` ifadeleriyle çakışması; Linux logunda `Failed to find wayland-scanner` görüldü.

## Testle yeniden üretilip düzeltilen hatalar

Düzeltmelerden önce genişletilmiş 18 testin 8'i başarısızdı; orijinal 9 test bu hataları yakalamıyordu.

| Sorun | Önceki davranış | Düzeltme / regresyon |
|---|---|---|
| Sphere-box normal yönü | Zemindeki küre yukarı çıkmak yerine içeri itiliyordu; aşağı hız çözülmüyordu | A -> B normal sözleşmesi, her iki çift sırası için dünya testi |
| Küre kutu içinde | Minimum mesafe yapay epsilon ile büyütüldüğünden iç yüz testi erişilemiyor, sıfır normal oluşuyordu | Gerçek uzaklık, en yakın yüze çıkış ve döndürülmüş kutu testi |
| Eş merkezli küreler | Normal sıfır, penetration yapay olarak küçülüyordu | Deterministik birim normal ve doğru penetration |
| Broadphase çift sırası | Unordered map/ekleme sırası solver sırasını etkiliyordu | Kanonik çiftler ve sıralama; ters ekleme testi |
| Render gecikmesi | Sekiz adımlık limit sonrasında backlog sonraki karelerde birikiyordu | Fazla tam tickleri atıp kesri koruma |
| Geçersiz dt | Negatif/NaN/Inf accumulator'ı bozabiliyordu | Dünya değişmeden `invalid_argument`; geçerli çağrı sonrası toparlanma |
| Sahne reset | Eski simülasyon zamanı/temas verisi kalabiliyordu | Merkezi `reset()` ve reset regresyonu |
| Yay kuvveti | Her velocity iterasyonunda tekrar uygulanıyor, 8 iterasyonda 8 kat etki yapıyordu | Yay kuvvetlerini tick başına tek çağrıya ayırma |

## Ek uygulamalar

- Kutuları dünya AABB'leriyle grid'e ekleyerek geniş zeminin gereksiz hücre doldurmasını azaltma.
- Çekirdek ve örnek sahneler ayrı CMake kütüphaneleri; grafik olmadan derleme ve headless CLI.
- Geçersiz solver ayarı kontrolü, güncellenen spatial cell size'ın uygulanması, etkin statik inverse mass.
- Daha önce kullanılmayan friction/rotation ayarlarının çözüm yoluna bağlanması ve davranış testleri.
- Profiler zamanlarını son tick'te iterasyonlar boyunca toplama; solver süresindeki broad/narrowphase çifte sayımını kaldırma.
- `Space` duraklatma tuşunu uygulama; `Step Frame` ile render dt yerine tam bir fizik tick'i; sahne değişiminde drag durumunu temizleme.
- Retina picking için pencere koordinatları; UI üzerine taşınmış fare bırakıldığında mouse spring'i serbest bırakma.
- UI ve debug çizgilerini swap öncesinde çizme; macOS forward-compatible OpenGL isteği.
- Linux'ta Wayland varsayılanını kapatma (isteğe bağlı açılabilir); Windows'ta `NOMINMAX`; masaüstü matrisi yanında Linux sanitizer işi.

## Bilinen sınırlar

OBB tek temas noktalıdır; istif ve enerji korunumu henüz referans deneylerle doğrulanmamıştır. Çok büyük statik geometri için ayrı broadphase ve hücre bütçesi hâlâ gereklidir. Tam replay, platformlar arası determinism, AI ve dağıtım paketleri sonraki aşamalardır. Masaüstü etkileşimleri gerçek pencere ve cihaz üzerinde ayrıca denenmelidir. Detaylı kabul koşulları [ROADMAP.md](ROADMAP.md) içinde tutulur.
