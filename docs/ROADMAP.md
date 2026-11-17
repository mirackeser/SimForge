# SimForge geliştirme planı

Güncelleme: 17 Kasım 2026. Öncelik: gerçekçi fizik, gerçek render çıktısı, anlaşılır C++ ve denetlenebilir davranış.

## Aşamalar

| Aşama | Durum | Yapılan / kalan iş | Kabul koşulu |
|---|---|---|---|
| M1 — Çekirdek | Uygulandı | Grafiklerden ayrı fizik kütüphanesi, sabit tick, reset, ekransız runner | Üç platformda derleme/test; headless sanitizer |
| M2 — Fizik doğruluğu | Temeli uygulandı | Çok noktalı OBB yüzleri, kenar teması, birikimli impuls, saniye bazlı damping; sırada CCD ve tickler arası warm starting | Serbest düşme, momentum/enerji, duran kutu, istif ve damping referansları; hızlı cisimlerde tunneling testi sonraki iş |
| M3 — Görsel stüdyo | Temeli uygulandı | GGX malzemeler, gölge, taş/ahşap, metal küreler, sahne ve sağ panel; sırada daha ayrıntılı geometri ve aydınlatma | Gerçek motor görüntüsü, gerçek shader/GL çalışması; farklı cihazlarda görünüm ve hız kontrolü |
| M4 — Deney verisi | Başladı | CSV var; sahne JSON, sürümlü manifest, kayıt/yükleme ve deney karşılaştırması bekliyor | Kaydet/yükle eşdeğerliği, bozuk veri reddi, aynı binary/ayarlarla yeniden üretim |
| M5 — Sağlamlık ve performans | Temeli uygulandı | Sayısal doğrulama, hücre/çift/çizgi bütçeleri, stale handle koruması, sabit SHA bağımlılıklar; sırada otomatik statik analiz ve daha kapsamlı fuzz/ölçek testleri | Sınır testleri, sanitizer, temiz uyarı derlemesi; 100/500/1000 cisim için medyan/p95 ölçümü |
| M6 — Dağıtım | Planlandı | macOS/Windows/Linux paketleri, demo videosu, sürüm notları | Temiz cihazda açılış/kapanış, sürücü/Retina kontrolleri, tekrarlanabilir artifact |

## Sıradaki öncelikler

1. **Hızlı cisimler:** swept sphere/TOI tabanlı CCD; duvarı bir tick içinde geçen cisim için regresyon.
2. **İstif ve sürtünme:** kalıcı contact anahtarları ve tickler arası warm starting; uzun süreli yatay sapma, tilt ve penetration ölçümleri.
3. **Joint doğruluğu:** açısal konum düzeltmesi ve enerji/joint uzunluğu hata raporları; daha sert yaylar için kararlı soft constraint formülasyonu.
4. **Görsel kalite:** gerçek kenar pahları, daha ayrıntılı köprü/çerçeve geometrisi, çevre haritası ve daha yumuşak alan ışığı/gölge yaklaşımı. Referans görsel kalite hedefidir; tamamlanmış özelliklerin ekran görüntüsü değildir.
5. **Deney formatı:** şema sürümü, solver/seed/platform/commit metadata, geçici dünyada doğrulayıp atomik sahne değişimi.

## Fizik ve güvenlik sınırları

Mevcut motor ayrık zamanlıdır; büyük hızlarda CCD olmadan tunneling olabilir. Manifold/impulslar henüz tickler arasında taşınmaz. Temas konum düzeltmesi açısal değildir ve joint çözümü yaklaşık stabilizasyon kullanır. Enerji korunumu tüm sahneler için garanti edilmez. Bu sınırlar referans deneylerle ayrı ayrı azaltılacak.

Kalite kontrolleri ve uygulanan kaynak sınırları [QUALITY.md](QUALITY.md), gözlenen test/render sonuçları [VALIDATION.md](VALIDATION.md) içinde tutulur. AI/LLM entegrasyonu mevcut uygulama kapsamına dahil değildir.

## İlerleme düzeni

Bulgu -> kabul koşulu -> uygulama -> ilgili test/gerçek render -> diff/format kontrolü -> commit/push -> CI. `main` geçmişi yeniden yazılmaz. Yeni özellikten önce başarısız doğrulama düzeltilir. Doğrulanmamış performans, gerçekçilik veya güvenlik iddiası README'ye eklenmez.
