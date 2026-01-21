#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        # Mesafe eşikleri
        self.dist_thresh_front = 0.5  # Öndeki engel mesafesi
        self.dist_thresh_wall = 0.4   # İdeal duvar mesafesi
        
        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state = 0 # 0: Duvar Bul, 1: Sola Dön, 2: Duvarı Takip Et

    def scan_cb(self, msg):
        # Lidar verisini 5 bölgeye ayırıyoruz
        # Turtlebot3 Lidar 360 derece: 0=Arka, 90=Sağ, 180=Ön, 270=Sol (veya tam tersi konfigürasyona göre değişebilir)
        # Genellikle: 0=Ön, 90=Sol, 270=Sağ. Simülasyonda deneyerek emin olunmalı.
        # Turtlebot3 Waffle Pi standart: 0 derece ön taraf.
        
        # Ön (0-30 ve 330-360), Sol (30-90), Sağ (270-330) gibi basitleştiriyoruz
        # Lidar ranges array boyutu genelde 360'tır.
        
        size = len(msg.ranges)
        # Basit bölge ortalamaları (min değer alarak güvenlik sağlıyoruz)
        self.regions = {
            'front':  min(min(msg.ranges[0:20]), min(msg.ranges[340:359]), 10),
            'fleft':  min(min(msg.ranges[20:60]), 10),
            'left':   min(min(msg.ranges[60:100]), 10),
            'right':  min(min(msg.ranges[260:300]), 10),
            'fright': min(min(msg.ranges[300:340]), 10),
        }
        self.decide_action()

    def decide_action(self):
        msg = Twist()
        
        # Durum Makinesi
        # Durum 1: Önü kapalı -> Sola Dön
        if self.regions['front'] < self.dist_thresh_front:
            msg.angular.z = 0.5
            msg.linear.x = 0.0
        
        # Durum 2: Önü açık ama sağda duvar yok -> Duvarı Bulmak için Sağa Dönerek İlerle
        elif self.regions['right'] > 0.6: 
            msg.linear.x = 0.15
            msg.angular.z = -0.2
            
        # Durum 3: Önü açık, sağda duvar var ama çok yakın -> Sola açıl
        elif self.regions['right'] < 0.3:
            msg.linear.x = 0.15
            msg.angular.z = 0.2
            
        # Durum 4: Her şey yolunda, duvarı sağa alıp ilerle (Düzeltme hareketi)
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
