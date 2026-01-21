# KTÜN – Süpürge Robotu / Oda Bazlı Temizlik ve QR Doğrulama

## Platform
- ROS1 Noetic
- Gazebo
- TurtleBot3 (Waffle Pi)

---

## Proje Amacı
Bu projede bir süpürge robotu:
1. Ev ortamının haritasını SLAM ile çıkarır
2. Haritayı kaydeder
3. AMCL ile lokalize olur
4. Odaların giriş noktalarına sırayla gider
5. Her oda girişinde QR kod okuyarak doğrulama yapar
6. Doğrulanan odada zigzag (coverage) temizlik algoritması uygular
7. Temizlik sonunda rapor üretir

---

## Klasör Yapısı
launch/ → sim, slam, navigation ve görev yöneticisi
worlds/ → turtlebot3_house + QR modeller
models/ → QR poster modelleri
maps/ → SLAM ile oluşturulan harita
config/ → görev tanımı (mission.yaml)
src/ → QR Reader ve Task Manager node’ları
scripts/ → zigzag temizlik algoritması


---

## Kurulum

```bash
cd ~/catkin_ws/src
git clone https://github.com/Fulku/ktun_cleaning_robot.git
cd ..
catkin_make
source devel/setup.bash


