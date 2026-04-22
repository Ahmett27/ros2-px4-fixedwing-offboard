import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
import math
from mavros_msgs.msg import State
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import Thrust


class FixedWingPositionControl(Node):
    def __init__(self):
        super().__init__('fixed_wing_controller')

        self.declare_parameter('target_x', 300.0) 
        self.declare_parameter('target_y', 0.0)   
        self.declare_parameter('target_z', 200.0)  
        

        self.current_pose = None
        self.is_stabilizing_before_maneuver = False

        self.current_mode = "BILINMIYOR"

        # Manevra durum kontrolü
        self.is_maneuvering = False
        self.target_roll_rad = 0.0  # Radyan cinsinden hedef yatış açımız

        # __init__ içine şu değişkenleri ekle
        self.current_quat = None
        self.locked_yaw = 0.0
        self.locked_pitch = 0.0
        
        # OFFBOARD modu yüksek hız sever. Timer'ı 10Hz'den 50Hz'e (0.02 saniye) çekiyoruz.
        self.timer = self.create_timer(0.02, self.publish_control_commands)
        

        # MAVROS durumunu (modunu) dinleyen abone
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_callback, 
            10
        )




        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # Mesaj kaybolabilir, yeniden gönderme yok
            durability=DurabilityPolicy.VOLATILE,         # Geç bağlanan subscriber eski mesajları almaz
            history=HistoryPolicy.KEEP_LAST,              # Sadece son N mesajı sakla
            depth=1                                       # N=1, sadece en son mesaj
        )

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # Tek mesaj, tek topik! Senkronizasyon derdi yok.
        self.att_pub = self.create_publisher(
            AttitudeTarget, 
            '/mavros/setpoint_raw/attitude', 
            10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 


        self.local_pos_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile
        )

        # SAAT: Her 1.0 saniyede bir 'log_position' görevini tetikler
        self.log_timer = self.create_timer(1.0, self.log_position)
        
        self.init_sequence()

    def init_sequence(self):
        self.get_logger().info('Konum Kontrolu baslatiliyor...')
        
        self.set_px4_param('NAV_DLL_ACT', 0)      
        self.set_px4_param('NAV_RCL_ACT', 0)      
        self.set_px4_param('COM_RCL_EXCEPT', 4)   
        
        # PX4'ün MAVROS kumanda sinyallerini kabul etmesi için
        self.set_px4_param('COM_RC_IN_MODE', 1) 
        
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
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

        # YENİ EKLE: OFFBOARD'a geçtikten tam 15 saniye sonra taklayı başlat
        self.timer_start_maneuver = self.create_timer(15.0, self.prepare_for_maneuver)


    def prepare_for_maneuver(self):
        """Takla öncesi 3 saniye düz uçuş yap."""
        self.get_logger().info('Takla öncesi 3 saniye düz uçuşa geçiliyor...')
        self.timer_start_maneuver.cancel()
        
        # Düz uçuş bayrağını aktif et
        self.is_stabilizing_before_maneuver = True
        
        # 3 saniye sonra asıl taklayı başlat
        self.timer_start_stabilize = self.create_timer(3.0, self.start_maneuver_after_stabilize)

    def start_maneuver_after_stabilize(self):
        """Düz uçuş tamamlandı, taklayı başlat."""
        self.get_logger().info('Düz uçuş tamamlandı, takla başlatılıyor!')
        self.is_stabilizing_before_maneuver = False
        self.timer_start_stabilize.cancel()
        self.start_maneuver()   # Orijinal start_maneuver metodunu çağır

    def start_maneuver(self):
        if self.is_maneuvering or self.is_stabilizing_before_maneuver:
            return
            
        self.get_logger().info('OFFBOARD: Gövde Hızı (Body Rate) ile saf takla başlatılıyor!')
        self.is_maneuvering = True
        self.timer_start_maneuver.cancel()
        
        # Takla süresini 1.5 saniye yapıyoruz. Hızlı döneceği için irtifa kaybetmeyecek.
        self.timer_stop_maneuver = self.create_timer(5, self.stop_maneuver)

    def stop_maneuver(self):
        self.get_logger().info('Takla bitti, normal konum (Position) hedeflerine dönülüyor.')
        self.is_maneuvering = False
        self.is_stabilizing_before_maneuver = False
        self.timer_stop_maneuver.cancel()
        
        # Sonraki olası manevralar için açıyı sıfırlıyoruz
        self.target_roll_rad = 0.0

    def publish_control_commands(self):
        # HER ZAMAN bir PoseStamped gönder (OFFBOARD timeout'unu engellemek için)
        pos_msg = PoseStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.pose.position.x = self.get_parameter('target_x').value
        pos_msg.pose.position.y = self.get_parameter('target_y').value
        pos_msg.pose.position.z = self.get_parameter('target_z').value
        self.pos_pub.publish(pos_msg)

        # Manevra öncesi 3 saniye düz uçuş (body rate sıfır)
        if self.is_stabilizing_before_maneuver:
            att_msg = AttitudeTarget()
            att_msg.header.stamp = self.get_clock().now().to_msg()
            att_msg.header.frame_id = 'base_link'
            att_msg.type_mask = 192  # Body rate kontrolü
            att_msg.body_rate.x = 0.0
            att_msg.body_rate.y = 0.0
            att_msg.body_rate.z = 0.0
            att_msg.thrust = 0.0
            self.att_pub.publish(att_msg)
            # Pozisyon setpoint'i zaten yukarıda gönderiliyor, return yok




        # Eğer manevra yapıyorsak, ayrıca AttitudeTarget göndererek override et
        if self.is_maneuvering:
            att_msg = AttitudeTarget()
            att_msg.header.stamp = self.get_clock().now().to_msg()
            att_msg.header.frame_id = 'base_link'
            att_msg.type_mask = 192  # Body rate kontrolü, attitude ve thrust yoksay
            att_msg.body_rate.x = 12.0
            att_msg.body_rate.y = 0.0
            att_msg.body_rate.z = 0.0
            att_msg.thrust = 0.0
            self.att_pub.publish(att_msg)
                
        else:
            # --- NORMAL UÇUŞ DURUMU: PoseStamped Basıyoruz ---
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = self.get_parameter('target_x').value
            msg.pose.position.y = self.get_parameter('target_y').value
            msg.pose.position.z = self.get_parameter('target_z').value
            
            self.pos_pub.publish(msg)


    def euler_to_quaternion(self, roll, pitch, yaw):
        # Roll, Pitch ve Yaw değerlerini alıp Quaternion (qx, qy, qz, qw) listesi döndürür.
        # Gelen değerlerin radyan (radian) formatında olması gerekir!
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]
    
    

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position
        self.current_quat = msg.pose.orientation

    def state_callback(self, msg):
        
        self.current_mode = msg.mode

    def get_pitch_yaw_from_quaternion(self, q):
        if q is None:
            return 0.0, 0.0
        
        # Pitch (Yunuslama)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        # Yaw (Sapma/Yön)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return pitch, yaw
 

    def log_position(self):
        # Eğer cebimizde bir konum verisi varsa ekrana yazdır
        if self.current_pose is not None:
            x = self.current_pose.x
            y = self.current_pose.y
            z = self.current_pose.z
            # Ekrana yazdırma kısmına 'MOD' bilgisini de ekledik
            self.get_logger().info(f'RADAR: X={x:.1f} | Y={y:.1f} | İrtifa={z:.1f} | MOD: {self.current_mode}')

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