#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry

class SystematicCleaner:
    def __init__(self):
        rospy.init_node("room_cleaner", anonymous=True)
        
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/cleaning_path", Path, queue_size=10, latch=True)
        
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        
        # Başlangıçta 5 metre (boş) varsayıyoruz
        self.front_dist = 5.0
        self.left_dist = 5.0
        self.right_dist = 5.0
        
        self.duration = rospy.get_param("~duration", 20)
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.turn_state = 1 

    def scan_cb(self, msg):
        ranges = msg.ranges
        # Ön mesafe (0 derece)
        front_slice = ranges[0:15] + ranges[-15:]
        # Hatalı okumaları (0.0) ve sonsuzları filtrele
        valid_front = [x for x in front_slice if x > 0.1 and x < 10.0]
        if valid_front:
            self.front_dist = min(valid_front)
        
        # Yan kontroller (basit filtre)
        try:
            left_slice = [x for x in ranges[80:100] if x > 0.1]
            if left_slice: self.left_dist = min(left_slice)
            
            right_slice = [x for x in ranges[260:280] if x > 0.1]
            if right_slice: self.right_dist = min(right_slice)
        except:
            pass

    def odom_cb(self, msg):
        self.path.header.stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header = self.path.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def move_straight(self, speed, duration):
        twist = Twist()
        twist.linear.x = speed
        end_time = time.time() + duration
        while time.time() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)

    def turn_90(self, direction):
        twist = Twist()
        twist.angular.z = direction * 1.57 
        # 1.2 saniye dön (90 dereceyi tutturmak için)
        for _ in range(12): 
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.2)

    def run(self):
        start_time = time.time()
        rate = rospy.Rate(10)
        
        print(f"--- SISTEMATIK TARAMA BASLADI ({self.duration} SN) ---", flush=True)
        
        # Başlangıçta kısa bir bekle (Lidar verisi otursun)
        rospy.sleep(1.0)
        
        while not rospy.is_shutdown():
            if time.time() - start_time > self.duration:
                print("--- TEMIZLIK SURESI DOLDU ---", flush=True)
                break

            twist = Twist()
            
            # --- MANTIK GÜNCELLEMESİ ---
            # Mesafeyi 0.35 metreye düşürdük (Daha yakına girebilir)
            if self.front_dist > 0.35:
                twist.linear.x = 0.25
                self.cmd_pub.publish(twist)
            else:
                print(f"DUVAR ({self.front_dist:.2f}m)! U-Donusu yapiliyor...", flush=True)
                
                # 1. Önce biraz geri çekil (Sıkışmayı önler)
                self.move_straight(-0.15, 1.0)
                
                # 2. İlk Dönüş
                self.turn_90(self.turn_state)
                
                # 3. Yana Kayma (Eğer yan taraf boşsa)
                side_clear = (self.turn_state == 1 and self.left_dist > 0.4) or \
                             (self.turn_state == -1 and self.right_dist > 0.4)
                
                if side_clear:
                    self.move_straight(0.2, 1.0) # 1 saniye yana kay
                
                # 4. İkinci Dönüş
                self.turn_90(self.turn_state)
                
                # Bir sonraki dönüş için yön değiştir
                self.turn_state *= -1 
                
            rate.sleep()

        self.cmd_pub.publish(Twist())

if __name__ == "__main__":
    try:
        SystematicCleaner().run()
    except rospy.ROSInterruptException:
        pass
