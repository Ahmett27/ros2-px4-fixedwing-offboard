import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
from mavros_msgs.msg import State, AttitudeTarget
import math

class FixedWingPositionControl(Node):
    def __init__(self):
        super().__init__('fixed_wing_controller')

        self.declare_parameter('target_x', 300.0) 
        self.declare_parameter('target_y', 0.0)   
        self.declare_parameter('target_z', 70.0)  

        self.current_pose = None
        self.current_mode = "BILINMIYOR"
        
        # Görev durumumuzu takip etmek için: 'INIT', 'FLY_TO_TARGET', 'ROLLING'
        self.mission_state = 'INIT'
        self.roll_start_time = None

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # OFFBOARD modunda kalabilmek için duruma göre Konum veya Yönelim (Attitude) yayınlayacağız
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.att_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', qos_profile)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 

        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile
        )

        # OFFBOARD modunun kapanmaması için saniyede 20 kez (0.05s) sürekli yayın yapan ana döngü
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Radarı saniyede 1 kez ekrana yazdır
        self.log_timer = self.create_timer(1.0, self.log_position)
        
        self.init_sequence()

    def init_sequence(self):
        self.get_logger().info('Konum Kontrolu baslatiliyor...')
        
        self.set_px4_param('NAV_DLL_ACT', 0)      
        self.set_px4_param('NAV_RCL_ACT', 0)      
        self.set_px4_param('COM_RCL_EXCEPT', 4)   
        
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
        # 10 saniye sonra Offboard'a geç
        self.timer_offboard = self.create_timer(10.0, self.switch_to_offboard)

    def set_px4_param(self, param_id, integer_value=0):
        req = ParamSet.Request()
        req.param_id = param_id
        req.value.integer = integer_value
        self.param_client.call_async(req)

    def switch_to_offboard(self):
        self.get_logger().info('OFFBOARD moduna geciliyor, hedefe gidiliyor (Position Control)...')
        self.mission_state = 'FLY_TO_TARGET'
        req = SetMode.Request(custom_mode='OFFBOARD')
        self.mode_client.call_async(req)
        self.timer_offboard.cancel()

        # Offboard uçuş başladıktan 12 saniye sonra taklayı başlat
        self.timer_start_roll = self.create_timer(12.0, self.start_roll)

    def start_roll(self):
        self.get_logger().info('ATTITUDE MESAJI: Sag tarafa 360 derece takla (Roll) baslatiliyor!')
        self.mission_state = 'ROLLING'
        self.roll_start_time = self.get_clock().now()
        self.timer_start_roll.cancel()

    def control_loop(self):
        # Duruma göre sürekli olarak Position veya Attitude setpoint göndermeliyiz
        
        if self.mission_state in ['INIT', 'FLY_TO_TARGET']:
            # Hedefe normal uçuş
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = self.get_parameter('target_x').value
            msg.pose.position.y = self.get_parameter('target_y').value
            msg.pose.position.z = self.get_parameter('target_z').value
            self.pos_pub.publish(msg)

        elif self.mission_state == 'ROLLING':
            # Attitude (Yönelim) ile takla manevrası
            msg = AttitudeTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            # type_mask = 128 (IGNORE_ATTITUDE). Sadece roll dönüş hızını (body_rate) dikkate al.
            msg.type_mask = 128  
            
            # X ekseninde (Roll) saniyede 3.14 radyan (~180 derece) dönüş hızı veriyoruz
            msg.body_rate.x = 3.14  
            msg.body_rate.y = 0.0
            msg.body_rate.z = 0.0
            
            # Takla atarken motorun durmaması ve irtifa kaybetmemesi için thrust (gaz) değeri (%80)
            msg.thrust = 0.8  

            self.att_pub.publish(msg)

            # Ne kadar süre geçtiğini hesapla
            elapsed = (self.get_clock().now() - self.roll_start_time).nanoseconds / 1e9
            
            # 3.14 radyan/saniye hızla dönüyoruz. 2.5 saniyede tam taklayı bitirip düzeltmiş olacaktır.
            if elapsed > 2.5:
                self.get_logger().info('Takla tamamlandi, tekrar Position kontrolune (hedefe) donuluyor!')
                self.mission_state = 'FLY_TO_TARGET'

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def state_callback(self, msg):
        self.current_mode = msg.mode

    def log_position(self):
        if self.current_pose is not None:
            x = self.current_pose.x
            y = self.current_pose.y
            z = self.current_pose.z
            durum = "HEDEFE UCUS" if self.mission_state == 'FLY_TO_TARGET' else ("TAKLA ATIYOR" if self.mission_state == 'ROLLING' else "BEKLIYOR")
            self.get_logger().info(f'RADAR: X={x:.1f} | Y={y:.1f} | Irtifa={z:.1f} | MOD: {self.current_mode} | DURUM: {durum}')

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