# SimForge geliştirme planı

İnceleme tarihi: 17 Kasım 2026. Başlangıç: `304ee95`.

## Ürün hedefi ve kapsam

SimForge bugün C++17 ile yazılmış, küre ve OBB rijit cisimlerini çalıştıran bir masaüstü fizik sandbox'ıdır. AI modeli, ajan orkestrasyonu veya doğal dil arayüzü henüz yoktur. İlk hedef: davranışı ölçülebilen, ekransız da çalışabilen, test edilmiş bir simülasyon çekirdeği. Sonraki hedef: kullanıcının fizik deneyini tanımladığı, sistemin güvenli bir sahne önerdiği ve sonuçları ölçümlerle açıkladığı **SimForge AI deney asistanı**.

Başarı ölçütü özellik sayısı değil; aynı deneyin yeniden çalıştırılabilmesi, fizik hatalarının yakalanması ve AI açıklamalarının gerçek simülasyon verileriyle desteklenmesidir.

## Aşamalar ve kabul koşulları

| Aşama | Durum | Teslimat | Kabul koşulu |
|---|---|---|---|
| M1 — Güvenilir çekirdek | İlk paket uygulandı | Grafiklerden bağımsız `SimForgePhysics`, ekransız runner, çarpışma/zaman/yay düzeltmeleri, regresyon testleri, CI düzeltmeleri | Ekransız configure/build/test geçer; masaüstü üç platformda CI ile doğrulanır; zemindeki küre dışarı itilir; yay kuvveti iterasyon sayısıyla katlanmaz |
| M2 — Deney ve veri altyapısı | Başladı: sabit tick + CSV | Sürümlü sahne JSON şeması, seed, ayar anlık görüntüsü, kayıt/yükleme, deney manifesti | Kaydet/yükle eşdeğerliği; bozuk veri reddi; aynı binary ve girdilerle aynı iz; platformlar arası karşılaştırmada tanımlı tolerans |
| M3 — Fizik doğruluğu ve performans | Planlandı | Referans deneyler, çok noktalı OBB temasları, warm starting, zaman ölçekli damping, statik geometri optimizasyonu | Serbest düşme ve elastik çarpışmada analitik referans; istif sapması ve joint hatası raporu; 100/500/1000 cisimde ölçülmüş broadphase/solver maliyeti |
| M4 — Deney çalışma alanı | Planlandı | Sahne editörü, undo/redo, seçili nesne parametreleri, deney karşılaştırması, grafikler | Kullanıcı bir deney oluşturur, kaydeder, yeniden açar ve iki koşulu karşılaştırır; Retina/picking ve duraklatma manuel kontrol edilir |
| M5 — AI deney asistanı | Planlandı; M2/M3'e bağlı | Doğal dil -> doğrulanan deney önerisi; ölçümlere dayalı açıklama; sağlayıcıdan bağımsız model arabirimi | AI yalnızca tipli/sınırlı komut önerir; uygulamadan önce önizleme; belirsizlik belirtilir; hatalı şema/taşma/uydurulmuş ölçüm testleri geçer |
| M6 — Portföy sürümü | Planlandı | Paketlenmiş macOS/Windows/Linux sürümleri, örnek deneyler, kısa demo, benchmark ve teknik rapor | Temiz makinede kurulum; artifact kontrolleri; sürüm notları ve tekrar üretme komutları; doğrulanmış özelliklerle README |

## Sıradaki iş paketi: M2

1. **Sahne sözleşmesi:** `schemaVersion`, entity kimliği, transform, collider, rigid body, joint, solver ayarı, seed. Geçersiz sayıları, kütleyi ve collider boyutlarını dosya sınırında reddet.
2. **Deney manifesti:** commit, derleyici/platform, timestep, iterasyon, sahne/seed ve çıktı şeması. CSV tek başına tam replay kaydı sayılmaz.
3. **Scene I/O:** geçici dünya üzerinde doğrula; yalnızca başarılıysa aktif sahneyle değiştir. Yuvarlak dönüş ve bozuk dosya testlerini ekle.
4. **Runner genişletmesi:** `--scene-file`, `--seed`, `--manifest`, deterministik checksum; çekirdekte AI/ağ bağımlılığı oluşturma.
5. **Referans deneyler:** serbest düşme, tek küre/zemin, iki küre momentumu, sarkaç. Konum/hız/enerji ve joint uzunluk hatası için ölçütleri tanımla.

## Teknik borç ve risk sırası

- **P1: OBB temasları.** Şu an tek temas noktası yaklaşımı kullanılır. Jenga gibi istiflerde doğruluk/stabilite kanıtlanmış değildir. Çok noktalı manifold ve accumulated impulse tasarımı birlikte ele alınmalı.
- **P2: Büyük statik geometri.** İlk pakette OBB dünya AABB'siyle grid yerleştirmesi eklendi. Daha büyük/sınırsız sahneler için statik broadphase ayrımı ve hücre bütçesi hâlâ değerlendirilmelidir.
- **P1: Kimlik yaşam döngüsü.** `World::clear()` kimlikleri yeniden kullanır; dış handle ve uzun süreli deney referansları için generation/index yapısı gerekir. `entitiesMutable()` invalidasyon kuralları açıkça tanımlanmalı.
- **P2: Damping.** Mevcut damping substep başına çarpandır; timestep değiştirilince fiziksel sönüm değişir. Saniye bazlı model ve sürüm geçişi tasarlanmalı.
- **P2: Ray başlangıcı.** Collider içinden başlayan raylerin normal/mesafe sözleşmesi belirlenip uç durumlar test edilmeli.
- **P2: Grafik kaynakları.** Başlatma hatası ve kapanışta shader/context yaşam döngüsü, küçültülmüş pencere ve giriş yakalama akışı için masaüstü testleri tamamlanmalı.
- **AI kapsam riski.** Önce deney sözleşmesi ve ölçüm, sonra model. AI'nin keyfi C++/Python üretip çalıştırmasına dayanan bir yol kullanılmayacak. Model fizik çözücünün yerini almayacak.

## Çalışma ve GitHub ilerleme düzeni

Her paket: bulgu -> kabul koşulu -> uygulama -> ilgili testler -> diff kontrolü -> commit/push -> CI sonucu. Bir sonraki paket bu dosyadaki durumları günceller. `main` geçmişi yeniden yazılmaz; eşzamanlı uzak değişiklikler varsa önce incelenir. CI başarısızsa bir sonraki özellikten önce neden düzeltilir. Model/API anahtarı gerektiren gelecekteki AI entegrasyonunda gizli değerler depoya yazılmaz.

Yerel doğrulama sonuçları ve ölçüm sınırları [VALIDATION.md](VALIDATION.md) dosyasındadır.

## Bu paketin doğrulama sınırı

Birim ve entegrasyon testleri hedefli davranışları ölçer; üretim motoru doğruluğu veya platformlar arasında bit düzeyinde deterministik sonuç garantisi vermez. Ekransız aynı-binary tekrar testi fizik çıktısına uygulanır; duvar saati profiler süreleri karşılaştırma verisine dahil edilmez. AI aşamaları tasarımdır ve henüz uygulanmamıştır.
