import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
import math
from mavros_msgs.msg import State
from mavros_msgs.msg import ManualControl

class FixedWingPositionControl(Node):
    # __init__ fonksiyonuna drone_id parametresi ekliyoruz. Varsayılan olarak 'uav1' verdik.
    def __init__(self, drone_id='uav1'):
        # Düğüm adının ROS ağında çakışmaması için düğüm adının başına drone_id ekliyoruz
        super().__init__(f'{drone_id}_fixed_wing_controller')

        self.drone_id = drone_id

        self.declare_parameter('target_x', 300.0) 
        self.declare_parameter('target_y', 0.0)   
        self.declare_parameter('target_z', 70.0)  

        self.current_pose = None
        self.current_mode = "BILINMIYOR"
        # Uçağın ne aşamada olduğunu takip etmek için bir "Durum" değişkeni
        self.mission_state = "YERDE"
        self.roll_cmd = 0.0 

        # Launch dosyasından gelen drone_id parametresini oku
        self.declare_parameter('drone_id', 'uav1')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value        

        # 1. DEĞİŞİKLİK: Topic isminin başına f'/{self.drone_id}' ekledik
        self.state_sub = self.create_subscription(
            State, 
            f'/{self.drone_id}/state', 
            self.state_callback, 
            10
        )
        
        
        self.rc_pub = self.create_publisher(ManualControl, f'/{self.drone_id}/manual_control/send', 10)
        
        # Otopilotun kumanda bağlantısının koptuğunu sanmaması için saniyede 20 kez sinyal basıyoruz (0.05 saniye)
        self.rc_timer = self.create_timer(0.05, self.publish_fake_rc)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # Mesaj kaybolabilir, yeniden gönderme yok
            durability=DurabilityPolicy.VOLATILE,         # Geç bağlanan subscriber eski mesajları almaz
            history=HistoryPolicy.KEEP_LAST,              # Sadece son N mesajı sakla
            depth=1                                       # N=1, sadece en son mesaj
        )

        self.pos_pub = self.create_publisher(PoseStamped, f'/{self.drone_id}/setpoint_position/local', qos_profile)

        self.arming_client = self.create_client(CommandBool, f'/{self.drone_id}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'/{self.drone_id}/set_mode')
        self.param_client = self.create_client(ParamSet, f'/{self.drone_id}/param/set') 

        self.timer = self.create_timer(0.1, self.publish_position)

        self.local_pos_sub = self.create_subscription(
            PoseStamped, 
            f'/{self.drone_id}/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile
        )

        # SAAT: Her 1.0 saniyede bir 'log_position' görevini tetikler
        self.log_timer = self.create_timer(1.0, self.log_position)
        
        # ESKİ HALİ: self.init_sequence() 
        # YENİ HALİ:
        self.startup_timer = self.create_timer(10.0, self.check_mavros_ready)




    def check_mavros_ready(self):
        # Artık True/False diye sormuyoruz! 10 saniye bekledik, otopilot kendine geldi.
        self.get_logger().info('Sistem hazir! Sürü İHA kalkis sekansi baslatiliyor...')
        self.startup_timer.cancel() # Saati durdur
        self.init_sequence()        # Kalkışı zorla başlat!

    def init_sequence(self):
        self.set_px4_param('NAV_DLL_ACT', 0)      
        self.set_px4_param('NAV_RCL_ACT', 0)      
        self.set_px4_param('COM_RCL_EXCEPT', 4)   
        self.set_px4_param('COM_RC_IN_MODE', 1) 
        
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
        # Kalkış emri verildi, durumu güncelle! (ZAMANLAYICILARI SİLDİK)
        self.mission_state = "KALKIS_YAPIYOR"

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

        # YENİ EKLE: Offboard'a geçtikten 12 saniye sonra çalışacak yeni zamanlayıcı
        self.timer_altitude = self.create_timer(12.0, self.switch_to_altitude)

    def switch_to_altitude(self): 
        self.get_logger().info('12 saniye doldu, ACRO (Akrobasi) moduna geçiliyor...')
        req = SetMode.Request(custom_mode='ACRO') 
        self.mode_client.call_async(req)
        self.timer_altitude.cancel()

        # ACRO'ya geçtikten 2 saniye sonra taklayı tetikler.
        self.timer_start_roll = self.create_timer(2.0, self.start_roll)

    def publish_position(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.get_parameter('target_x').value
        msg.pose.position.y = self.get_parameter('target_y').value
        msg.pose.position.z = self.get_parameter('target_z').value
        self.pos_pub.publish(msg)

    def pose_callback(self, msg):
        
        self.current_pose = msg.pose.position

    def state_callback(self, msg):
        
        self.current_mode = msg.mode

    def publish_fake_rc(self):
        msg = ManualControl()
        msg.x = 0.0            # Pitch (Burun Aşağı/Yukarı) -> 0.0 Düz
        msg.y = self.roll_cmd  # Roll (Sağ/Sol Yatış)
        msg.z = 800.0          # Throttle (Gaz) -> %80 güç
        
        self.rc_pub.publish(msg)

    def start_roll(self):
        self.get_logger().info('AKROBASİ: Sağ tarafa 360 derece takla (Roll) başlatılıyor!')
        # Tam sağ komutu (Maksimum 1000.0)
        self.roll_cmd = 1000.0  
        self.timer_start_roll.cancel()
        
        self.timer_stop_roll = self.create_timer(5.5, self.stop_roll)

    def stop_roll(self):
        self.get_logger().info('AKROBASİ: Takla tamamlandı, otopilot (OFFBOARD) uçağı toparlıyor!')
        
        # Dönme hızını kesiyoruz
        self.roll_cmd = 0.0  
        self.timer_stop_roll.cancel()

        # Uçağın kendini dengeleyip hedefe uçmaya devam etmesi için hemen OFFBOARD'a dönüyoruz
        req = SetMode.Request(custom_mode='OFFBOARD')
        self.mode_client.call_async(req)

    def log_position(self):
        if self.current_pose is not None:
            x = self.current_pose.x
            y = self.current_pose.y
            z = self.current_pose.z
            
            # Ekrana durumu da yazdıralım
            self.get_logger().info(f'RADAR: Irtifa={z:.1f}m | MOD: {self.current_mode} | GOREV: {self.mission_state}')

            # AKILLI KONTROL (Zaman yerine İrtifa kontrolü)
            if self.mission_state == "KALKIS_YAPIYOR":
                # Eğer 30 metreye ulaştıysa (simülasyon ne kadar kasarsa kassın bekler)
                if z > 30.0:  
                    self.get_logger().info('30 Metreye ulaşıldı! OFFBOARD moduna geçiliyor...')
                    req = SetMode.Request(custom_mode='OFFBOARD')
                    self.call_service(self.mode_client, req)
                    self.mission_state = "TIRMANIYOR" # Durumu güncelle
            
            elif self.mission_state == "TIRMANIYOR":
                # Eğer 60 metreye ulaştıysa taklaya hazırlan
                if z > 60.0:  
                    self.get_logger().info('60 Metreye ulaşıldı! ACRO (Akrobasi) moduna geçiliyor...')
                    req = SetMode.Request(custom_mode='ACRO')
                    self.call_service(self.mode_client, req)
                    
                    # Taklayı başlatmak için 2 saniye bekle
                    self.timer_start_roll = self.create_timer(2.0, self.start_roll)
                    self.mission_state = "TAKLA_ATIYOR"

    def call_service(self, client, request):
        # "Servis hazır mı?" engelini kaldırdık. Adresi biliyoruz, komutu direkt basıyoruz!
        client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingPositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()