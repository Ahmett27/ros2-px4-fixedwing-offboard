import rclpy
from rclpy.node import Node
# DİKKAT: VOLATILE eklendi
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
import math

class MultiWaypointAutonomousControl(Node):
    def __init__(self):
        super().__init__('planned_flight_controller')

        self.waypoints = [
            [150.0, 0.0, 70.0],    
            [150.0, 150.0, 70.0],  
            [0.0, 150.0, 80.0],    
            [0.0, 0.0, 70.0]       
        ]
        
        self.current_wp_index = 0 
        self.current_pose = None  

        # 1. ÇÖZÜM: QoS POLİTİKASI DÜZELTİLDİ (VOLATILE yapıldı)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # MAVROS'un istediği ayar budur
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 

        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 2. ÇÖZÜM: Node'un bloke olmasını engellemek için init_sequence'ı 2 saniye gecikmeli başlatıyoruz.
        # Bu sayede rclpy.spin() devreye girer ve ağdaki servisler (ParamSet) keşfedilir.
        self.init_timer = self.create_timer(2.0, self.init_sequence)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def set_px4_param(self, param_id, integer_value=0):
        # Servis çağrısı
        req = ParamSet.Request()
        req.param_id = param_id
        req.value.integer = integer_value
        
        self.param_client.call_async(req)
        self.get_logger().info(f'PARAMETRE GÖNDERİLDİ: {param_id} -> {integer_value}')

    def init_sequence(self):
        # Başlangıç zamanlayıcısını iptal et ki bu fonksiyon tekrar tekrar çalışmasın
        self.init_timer.cancel()
        
        self.get_logger().info('OTONOM GÖREV BAŞLIYOR: Failsafe ayarları eziliyor...')
        
        # Servislerin hazır olduğundan emin ol (Burada spin aktif olduğu için anında bulacaktır)
        if self.param_client.wait_for_service(timeout_sec=3.0):
            self.set_px4_param('NAV_DLL_ACT', 0)      
            self.set_px4_param('NAV_RCL_ACT', 0)      
            self.set_px4_param('COM_RCL_EXCEPT', 4)   
        else:
            self.get_logger().error('HATA: Parametre servisi bulunamadı!')

        self.call_service(self.arming_client, CommandBool.Request(value=True))
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
        self.timer_offboard = self.create_timer(25.0, self.switch_to_offboard)

    def switch_to_offboard(self):
        self.get_logger().info('Süre doldu, OFFBOARD moduna geçiliyor...')
        req = SetMode.Request(custom_mode='OFFBOARD')
        self.mode_client.call_async(req)
        self.timer_offboard.cancel()

    def control_loop(self):
        if self.current_wp_index >= len(self.waypoints):
            target = self.waypoints[-1] 
        else:
            target = self.waypoints[self.current_wp_index]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        self.pos_pub.publish(msg)

        if self.current_pose is not None and self.current_wp_index < len(self.waypoints):
            dist = math.sqrt(
                (target[0] - self.current_pose.x)**2 + 
                (target[1] - self.current_pose.y)**2 + 
                (target[2] - self.current_pose.z)**2
            )

            if dist < 15.0:
                self.get_logger().info(f'WAYPOINT {self.current_wp_index + 1} TAMAMLANDI!')
                self.current_wp_index += 1

    def call_service(self, client, request):
        if client.wait_for_service(timeout_sec=3.0):
            client.call_async(request)
        else:
            self.get_logger().error('HATA: MAVROS servisine ulaşılamadı.')

def main(args=None):
    rclpy.init(args=args)
    node = MultiWaypointAutonomousControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()