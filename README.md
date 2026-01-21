# Süpürge Robotu Projesi: Oda Bazlı Temizlik ve QR Doğrulama (Waypoint Kullanmadan)

**Hazırlayan:** Feyza Ülkü Öztaşkın
**Platform:** ROS1 Noetic + TurtleBot3 (Gazebo)


---

## 1. Proje Tanımı ve Amacı

Bu proje, **KTÜN Robotiğe Giriş dersi final uygulama ödevi** kapsamında geliştirilmiştir. Projenin temel amacı; Gazebo simülasyon ortamında çalışan bir **TurtleBot3 mobil robotunun**, ev ortamının haritasını çıkardıktan sonra (SLAM) belirli odalara otonom olarak gitmesi, her oda girişinde bulunan **QR kodlar** aracılığıyla doğru odada olduğunu doğrulaması ve doğrulama başarılı ise o oda içerisinde **önceden tanımlı waypoint kullanmadan**, sensör verilerine dayalı **alan kapsama (coverage) temelli temizlik algoritması** ile tüm odayı dolaşmasıdır.

Bu proje; gerçek süpürge robotlarının çalışma prensiplerini esas alarak, mobil robotlarda **haritalama, lokalizasyon, navigasyon, görüntü işleme, görev planlama (task management)** ve **reaktif hareket** yaklaşımlarını bütünleşik bir sistem halinde sunmaktadır.

---

## 2. Kullanılan Yöntem ve Teknolojiler

Projede aşağıdaki yazılım ve teknolojiler kullanılmıştır:

* **İşletim Sistemi:** Ubuntu 20.04 LTS
* **Robot Orta Katmanı:** ROS1 Noetic
* **Simülasyon Ortamı:** Gazebo 11 – TurtleBot3 (Burger modeli)
* **Haritalama (SLAM):** `gmapping`
* **Lokalizasyon:** `amcl`
* **Navigasyon:** `move_base` (sadece oda girişlerine ulaşmak için)
* **Oda İçi Temizlik:** Waypoint kullanmadan çalışan, LIDAR tabanlı alan kapsama algoritması
* **Görüntü İşleme:** `cv_bridge`, `OpenCV`, `pyzbar` (QR kod okuma)
* **Programlama Dili:** Python 3 (`rospy`)

---

## 3. Sistem Mimarisi

Sistem, ROS düğümleri ve launch dosyaları üzerinden modüler bir yapı ile tasarlanmıştır. Ana bileşenler aşağıda özetlenmiştir:

* **Gazebo Simülasyonu:** TurtleBot3 ev ortamı ve odalara yerleştirilmiş QR kodlar
* **SLAM ve Harita Kaydı:** Ortamın keşfedilmesi ve kalıcı haritanın oluşturulması
* **Navigation Stack:** AMCL tabanlı lokalizasyon ve oda girişlerine otonom ulaşım
* **QR Okuyucu Node:** Kamera görüntüsünden QR kod tespiti ve doğrulama
* **Görev Yöneticisi (Task Manager):** Robotun tüm görev akışını yöneten durum makinesi
* **Coverage Temizlik Modu:** Oda içerisinde waypoint kullanmadan alan tarama algoritması

---

## 4. Kurulum Talimatları

### 4.1 Çalışma Alanının Hazırlanması

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Fulku/ktun_cleaning_robot.git
```

### 4.2 Gerekli Paketlerin Yüklenmesi

```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-navigation
sudo apt install ros-noetic-cv-bridge python3-pip
pip3 install pyzbar opencv-python
```

TurtleBot3 modeli tanımlanır:

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 4.3 Derleme

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 5. Çalıştırma Adımları

Sistem, her alt bileşen kontrol edilebilir olacak şekilde ayrı launch dosyalarıyla çalıştırılmaktadır.

### 5.1 Gazebo Simülasyonu

```bash
roslaunch ktun_cleaning_robot sim.launch
```

### 5.2 Navigasyon (AMCL + move_base)

```bash
roslaunch ktun_cleaning_robot nav.launch
```

Bu aşamada RViz üzerinden **2D Pose Estimate** aracı ile robotun başlangıç konumu belirlenir.

### 5.3 Görev Yöneticisi ve QR Okuyucu

```bash
roslaunch ktun_cleaning_robot task_manager.launch
```

---

## 6. Algoritma ve Senaryo Akışı

Robotun tüm davranışları `task_manager.py` dosyası içerisinde tanımlanmış bir **durum makinesi (State Machine)** ile yönetilmektedir.

> **Not:** Bu projede **oda içi temizlik için kesinlikle waypoint kullanılmamıştır**.

### Durumlar:

1. **INIT:** Harita ve navigasyon sistemlerinin hazır olup olmadığı kontrol edilir.
2. **GO_TO_ROOM_ENTRY:** Robot, sıradaki odanın giriş noktasına `move_base` aracılığıyla gönderilir.
3. **QR_VERIFY:** Kamera görüntüsünden QR kod okunur.

   * Beklenen QR (ör. `ROOM=KITCHEN`) algılanırsa doğrulama başarılıdır.
   * QR okunamazsa robot küçük açı ve mesafe düzeltmeleri yaparak tekrar dener.
4. **EXECUTE_CLEANING (Coverage Mode):**

   * Robot odaya giriş yapar.
   * LIDAR verisine dayalı olarak **duvar takip** ve **zigzag benzeri kapsama** mantığıyla tüm alanı tarar.
   * Engel algılandığında yön değiştirir ve kapsama oranını artıracak şekilde hareket eder.
5. **REPORT:** Odaya ait görev sonucu (SUCCESS / SKIPPED / FAIL) kaydedilir.
6. **NEXT_ROOM:** Bir sonraki odaya geçilir.
7. **FINISH:** Tüm odalar tamamlandığında genel temizlik raporu oluşturulur.

---

## 7. Görev Tanımları (Waypoint Kullanmadan)

Görev tanımları `config/mission.yaml` dosyasında tutulmaktadır. Bu dosyada **yalnızca oda sırası, oda giriş koordinatları ve beklenen QR bilgisi** bulunmaktadır.

Oda içerisine girildikten sonra robot, bu dosyada tanımlı herhangi bir waypoint’e gitmez; temizlik tamamen reaktif kapsama algoritması ile yapılır.

Örnek `mission.yaml` yapısı:

```yaml
rooms: [LIVINGROOM, KITCHEN, BEDROOM, CORRIDOR]

LIVINGROOM:
  entry_goal: {x: 1.2, y: 0.5, yaw: 1.57}
  qr_expected: "ROOM=LIVINGROOM"
```

---

## 8. Dosya Yapısı

```text
ktun_cleaning_robot/
├── launch/        # Simülasyon, SLAM, navigasyon ve görev yöneticisi launch dosyaları
├── worlds/        # QR kodlar eklenmiş Gazebo ev ortamı
├── maps/          # SLAM ile oluşturulan map.yaml ve map.pgm
├── config/        # mission.yaml (oda girişleri ve QR bilgileri)
├── models/        # QR kod texture ve model dosyaları
├── src/
│   ├── qr_reader.py       # QR okuma düğümü
│   └── task_manager.py   # Görev yöneticisi ve coverage algoritması
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 9. Hata Yönetimi

* **QR Okunamazsa:** Robot küçük konum ve açı düzeltmeleri ile 2–3 kez tekrar dener, başarısız olursa oda `SKIPPED` olarak işaretlenir.
* **Navigasyon Hatası:** Oda giriş noktasına ulaşılamazsa bir kez yeniden denenir, yine başarısız olursa `FAIL` olarak raporlanır.
* **Timeout:** Her oda için maksimum süre sınırı tanımlanmıştır.

---

## 10. Sonuç ve Değerlendirme

Bu proje kapsamında TurtleBot3 robotu;

* Ortamın haritasını SLAM ile başarıyla çıkarmış ve kaydetmiş,
* Harita üzerinde AMCL ile güvenilir şekilde lokalize olmuş,
* En az dört farklı oda için giriş noktalarına otonom olarak gitmiş,
* QR kodlar aracılığıyla oda doğrulaması yapmış,
* **Oda içi temizlik görevlerini önceden tanımlı waypoint kullanmadan**, sensör tabanlı kapsama algoritması ile gerçekleştirmiş,
* Tüm sürecin sonunda detaylı bir temizlik raporu üretmiştir.

---

