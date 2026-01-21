#!/usr/bin/env python3
import rospy
import yaml
import actionlib
import subprocess
import os
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_srvs.srv import Empty # Costmap temizliği için

class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager")
        
        self.qr_data = None
        self.qr_time = rospy.Time(0)
        rospy.Subscriber("/qr_result", String, self.qr_cb)
        
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        
        # Costmap Temizleme Servisi (Navigasyon hatasını çözer)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        self.report = []
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('ktun_cleaning_robot')
        mission_path = os.path.join(package_path, "config", "mission.yaml")
        
        rospy.loginfo(f"Loading mission from: {mission_path}")
        
        if not os.path.exists(mission_path):
             rospy.logerr(f"CRITICAL ERROR: Mission file NOT found at {mission_path}")
             return
             
        with open(mission_path, "r") as f:
            self.mission_data = yaml.safe_load(f)

    def qr_cb(self, msg):
        self.qr_data = msg.data.strip()
        self.qr_time = rospy.Time.now()

    def go_to_entry(self, entry_pose):
        # Yola çıkmadan önce haritadaki hayalet engelleri temizle
        try:
            self.clear_costmaps()
            rospy.loginfo("Costmaps cleared for better navigation.")
        except:
            pass

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        pos = entry_pose['position']
        goal.target_pose.pose.position.x = pos['x']
        goal.target_pose.pose.position.y = pos['y']
        goal.target_pose.pose.position.z = pos['z']
        
        orient = entry_pose['orientation']
        goal.target_pose.pose.orientation.x = orient['x']
        goal.target_pose.pose.orientation.y = orient['y']
        goal.target_pose.pose.orientation.z = orient['z']
        goal.target_pose.pose.orientation.w = orient['w']
        
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(60))
        
        if not finished:
            self.client.cancel_goal()
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED


    def verify_qr_logic(self, expected, retries=30):
        rospy.loginfo(f"--- QR VERIFICATION START ---")
        rospy.loginfo(f"Expected QR: '{expected}'")
        

        rospy.sleep(2.0)
       
        
        for i in range(retries):
            if self.qr_data is None:
                rospy.logwarn(f"Try {i+1}/{retries}: Waiting for QR data...")
                rospy.sleep(1.0)
                continue
            
            if (rospy.Time.now() - self.qr_time).to_sec() > 3.0:
                 # Veri eskiyse bile son görüleni kabul etmeyi dene (esneklik)
                 pass

            rospy.loginfo(f"Try {i+1}/{retries}: Camera sees -> '{self.qr_data}'")

            if expected in self.qr_data:
                rospy.loginfo("!!! MATCH CONFIRMED !!!")
                return True
            else:
                rospy.logwarn(f"MISMATCH! Expected '{expected}' but saw '{self.qr_data}'")
            
            rospy.sleep(1.0)
            
        rospy.logerr("QR Verification TIMEOUT.")
        return False

    def run_cleaning(self, params):
        duration = params.get('duration', 20)
        rospy.loginfo(f"Starting ZigZag cleaning for {duration} seconds...")
        
        # python3 -u parametresi eklendi: Çıktıları anında terminale basar
        cmd = f"rosrun ktun_cleaning_robot room_cleaner.py _duration:={duration}"
   
        subprocess.call(cmd, shell=True)

    def generate_report(self):
        print("\n" + "="*40)
        print("      FINAL CLEANING REPORT      ")
        print("="*40)
        for room, status in self.report:
            print(f"{room:<15} : {status}")
        print("="*40 + "\n")

    def execute(self):
        if not hasattr(self, 'mission_data'): return

        rooms_list = self.mission_data['rooms']

        for room_info in rooms_list:
            room_name = room_info['name']
            rospy.loginfo(f"--- PROCESSING: {room_name} ---")
            
            if not self.go_to_entry(room_info['entry_pose']):
                rospy.logerr(f"{room_name} Navigation Failed!")
                self.report.append((room_name, "FAIL (Nav)"))
                continue
            
            expected_qr = room_info['qr_data']
            if self.verify_qr_logic(expected_qr):
                self.run_cleaning(room_info['cleaning_params'])
                self.report.append((room_name, "SUCCESS"))
            else:
                self.report.append((room_name, "SKIPPED (QR Fail)"))
        
        self.generate_report()

if __name__ == "__main__":
    try:
        TaskManager().execute()
    except rospy.ROSInterruptException:
        pass
