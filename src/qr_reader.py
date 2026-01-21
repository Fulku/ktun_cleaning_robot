#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

class QRReader:
    def __init__(self):
        rospy.init_node("qr_reader", anonymous=True)
        
        self.bridge = CvBridge()
        self.qr_pub = rospy.Publisher("/qr_result", String, queue_size=10)
        

        self.camera_topic = "/camera/rgb/image_raw"
        
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.callback)
        
        rospy.loginfo(f"QR Reader Baslatildi! '{self.camera_topic}' dinleniyor...")
        self.last_log_time = 0
        self.frame_count = 0

    def callback(self, data):
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            rospy.loginfo("Kamera goruntusu aliniyor... (Sistem aktif)")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Hatasi: {e}")
            return

        decoded_objects = decode(cv_image)
        
        # QR yoksa bile pencereyi güncelle (Donmayi engeller)
        if not decoded_objects:
            cv2.imshow("Robot Camera View", cv_image)
            cv2.waitKey(1)
            return

        for obj in decoded_objects:
            qr_data = obj.data.decode("utf-8")
            
            # Veriyi Yayınla
            msg = String()
            msg.data = qr_data
            self.qr_pub.publish(msg)
            
            # Ekrana Kırmızı Çerçeve Çiz
            points = obj.polygon
            if len(points) == 4:
                pts = np.array(points, dtype=np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(cv_image, [pts], True, (0, 0, 255), 3)
            
            # Metni Yaz
            cv2.putText(cv_image, qr_data, (obj.rect.left, obj.rect.top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Terminale yaz 
            if rospy.get_time() - self.last_log_time > 1.0:
                rospy.loginfo(f"QR BULUNDU VE YAYINLANDI: {qr_data}")
                self.last_log_time = rospy.get_time()

        cv2.imshow("Robot Camera View", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        QRReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
