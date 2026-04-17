import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
import math

class FixedWingPositionControl(Node):
    def __init__(self):
        super().__init__('fixed_wing_controller')

        self.declare_parameter('target_x', 300.0) 
        self.declare_parameter('target_y', 0.0)   
        self.declare_parameter('target_z', 70.0)  
        
        #Mevcut konumu takip etmek için bir değişken ekledim.
        self.current_pos = None


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 

        self.timer = self.create_timer(0.1, self.publish_position)



        #Konumu dinleyen abone (Dikkat: Koddaki mevcut qos_profile'ı kullanıyor!)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile
        )


        #SAAT: Her 2.0 saniyede bir 'log_position' görevini tetikler
        self.log_timer = self.create_timer(2.0, self.log_position)





        
        # Senin eski kodunda olduğu gibi beklemeden doğrudan çalıştırıyoruz
        self.init_sequence()

    def init_sequence(self):
        self.get_logger().info('L1 Algoritmasi icin Konum Kontrolu baslatiliyor...')
        
        # Simülasyonda kumanda yok diye kalkışı reddetmesini engelliyoruz
        self.set_px4_param('NAV_DLL_ACT', 0)      
        self.set_px4_param('NAV_RCL_ACT', 0)      
        self.set_px4_param('COM_RCL_EXCEPT', 4)   
        
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
        # Gerçek dünya saatiyle 10 saniye tırmandıktan sonra OFFBOARD'a geç
        self.timer_offboard = self.create_timer(10.0, self.switch_to_offboard)

    def set_px4_param(self, param_id, integer_value=0):
        req = ParamSet.Request()
        req.param_id = param_id
        req.value.integer = integer_value
        self.param_client.call_async(req)

    def switch_to_offboard(self):
        self.get_logger().info('OFFBOARD moduna geçiliyor, hedefe gidiliyor...')
        req = SetMode.Request(custom_mode='OFFBOARD')
        self.mode_client.call_async(req)
        self.timer_offboard.cancel()

    def publish_position(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.get_parameter('target_x').value
        msg.pose.position.y = self.get_parameter('target_y').value
        msg.pose.position.z = self.get_parameter('target_z').value
        self.pos_pub.publish(msg)



    def pose_callback(self, msg):
        # Telsizden gelen X, Y, Z bilgisini cebimize atıyoruz
        self.current_pose = msg.pose.position

    def log_position(self):
        # Eğer cebimizde bir konum verisi varsa ekrana yazdır
        if self.current_pose is not None:
            x = self.current_pose.x
            y = self.current_pose.y
            z = self.current_pose.z
            self.get_logger().info(f'RADAR: X={x:.1f} | Y={y:.1f} | İrtifa={z:.1f}')


    def call_service(self, client, request):
        if client.wait_for_service(timeout_sec=3.0):
            client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingPositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()