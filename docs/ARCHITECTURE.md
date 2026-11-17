# SimForge mimarisi

## Altı sorumluluk katmanı

| Katman | Bugünkü bileşen | Sorumluluk ve sonraki adım |
|---|---|---|
| 1. Sunum ve giriş | `Application`, `Renderer`, ImGui | Kullanıcı girdisi, 3D görüntü, inspector. Fizik adımını görüntü hızına göre değiştirmez. AI paneli ileride burada yer alır. |
| 2. Uygulama/deney akışı | `DemoScenes`, `SimForgeHeadless` | Sahne seçme, reset, sabit tick çalıştırma, CSV. M2'de deney manifesti ve kayıt/yükleme eklenir. |
| 3. Dünya ve durum | `World`, entity/component tipleri | Transform, kütle, collider, renk, etiket; renderer veya ağ bağımlılığı yok. Kalıcı handle sözleşmesi M2/M3 borcudur. |
| 4. Fizik çekirdeği | `PhysicsWorld`, collision, spatial hash, constraints | Sabit zaman, entegrasyon, aday çiftler, dar faz, impuls ve joint çözümleri. Tek doğruluk kaynağıdır. |
| 5. Veri ve ölçüm | CSV runner + `PhysicsProfileStats` | Fizik izleri ve substep süreleri. Manifest/şema/benchmark deposu planlandı. |
| 6. AI ve dış adaptörler | Henüz yok | Doğal dil isteği -> tipli deney önerisi -> doğrulama/önizleme -> çalıştırma -> ölçüme bağlı açıklama. Çekirdeğe doğrudan bağımlılık eklemez. |

```mermaid
flowchart TD
  UI["Desktop / ImGui"] --> Scenes["Sahne ve deney akışı"]
  CLI["Headless CLI"] --> Scenes
  Scenes --> Core["World + PhysicsWorld"]
  Core --> Trace["CSV ve profiler"]
  AI["AI adaptörü: planlandı"] -.-> Scenes
  Trace -.-> AI
```

## CMake hedefleri

- `SimForgePhysics`: ECS ve tüm fizik kaynakları; yalnızca GLM'e bağlı.
- `SimForgeScenes`: örnek sahneler; `SimForgePhysics` kullanır.
- `SimForgeHeadless`: pencere açmadan aynı sahneleri çalıştırır; CSV fizik izi üretir ve NaN/Inf durumda başarısız çıkar.
- `SimForge`: GLFW + GLAD + ImGui masaüstü uygulaması; aynı fizik kütüphanesine bağlanır.
- `SimForgeTests`: bu kütüphaneleri doğrudan test eder; fizik kaynaklarının ayrı bir kopyasını derlemez.

`SIMFORGE_BUILD_SANDBOX=OFF` ile GLFW, GLAD, ImGui ve OpenGL keşfi tamamen atlanır. Testler için GLM ve GoogleTest; runner için GLM gerekir. Bağımlılıklar sabit etiketlerle FetchContent üzerinden alınır; ilk configure ağ erişimi gerektirir.

## Fizik tick sözleşmesi

`step(world, frameDt)` render zamanını double accumulator'a ekler, en fazla sekiz sabit tick çalıştırır. Aşırı yükte fazladan tam tickler atılır, kesir korunur. Bu gerçek zamanlı etkileşimde kontrollü yavaşlama politikasıdır. Çevrimdışı deneyler zaman kaybetmemek için `stepFixed(world)` kullanır. Bu çağrı tam bir tick çalıştırır ve render accumulator'ına dokunmaz.

Bir tick sırası:

1. Yay ve mouse spring kuvvetleri bir kez uygulanır; hız ve transform entegrasyonu yapılır.
2. Her velocity iterasyonunda mesafe jointleri, broadphase, dar faz ve temas impulsları çözülür. Broadphase/temaslar her iterasyonda yeniden hesaplanır; manifold cache henüz yoktur.
3. Position iterasyonlarında mesafe joint hatası düzeltilir.
4. Simülasyon saati sabit dt kadar ilerler. Profiler değerleri son tick'in tüm iterasyon toplamlarıdır; constraint zamanı broadphase/dar fazı içermez.

Tüm temas normalleri A'dan B'ye yönelir; konum düzeltmesi A'yı `-normal`, B'yi `+normal` yönünde taşır. İç içe şekillerde bu ifade çözüm yönü sözleşmesidir. Statik cisimlerin etkin inverse mass'ı sıfırdır. `enableFriction` teğetsel impulsu kapatır. `enableRotationalDynamics=false` açısal entegrasyonu/impulsu durdurur; mevcut spin korunur ve tekrar açılınca devam eder.

`ConstraintSolver::applySpringForces` her tick'te bir kez, `solveVelocityConstraints` ise yalnızca mesafe kısıtları için iteratif çağrılır. Eski birleşik solver imzası bu iki sorumluluğa ayrılmıştır.

## Reset ve tekrarlanabilirlik

`loadDemoScene` dünyayı temizler ve `PhysicsWorld::reset()` çağırır: zaman, accumulator, joints, mouse spring, temaslar, grid ve profiler temizlenir. Solver ayarları korunur. Sahneye ait UI seçimi/drag durumu Application tarafından temizlenir.

Kutular grid'e döndürülmüş dünya AABB'leriyle eklenir; geniş zeminin küresel radius'u kadar gereksiz hacim doldurulmaz. Küreler radius tabanlı sınır kullanır. Aday çiftlerdeki bounding sphere filtresi korunur.

Broadphase çiftleri kanonik `(a < b)` sırada leksikografik olarak sıralanır. Böylece unordered map gezinme ve ekleme sırası solver sırasını belirlemez. Bu, farklı CPU/derleyiciler arasında aynı floating point bitlerini garanti etmez. Rastgele Partikül Yağmuru sahnesi sabit seed kullanır; masaüstü elle spawn işlemleri hâlâ rastgeledir.

CSV sütunları `step,time,entity,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz` şeklindedir. Tüm aktif cisimler her tick sonunda yazılır. CSV henüz sahne ayarlarını/şekilleri içeren bir replay formatı değildir.
