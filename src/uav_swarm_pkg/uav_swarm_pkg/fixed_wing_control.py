# ROS 2'nin Python ana kütüphanesini içe aktarıyoruz. Bu kütüphane ROS 2 sistemiyle iletişim kurmamızı sağlar.
import rclpy
# rclpy kütüphanesinin içinden Node (Düğüm) sınıfını alıyoruz. Kendi yazacağımız sınıf bu temel sınıftan miras alacak.
from rclpy.node import Node
# ROS 2'de mesajların iletim kalitesini (QoS - Quality of Service) ayarlamak için gereken profilleri ve kuralları içeri aktarıyoruz.
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# Hedef konumu (X, Y, Z) ve yönelimi bildirmek için standart geometri mesaj tipini (PoseStamped) içeri aktarıyoruz.
from geometry_msgs.msg import PoseStamped
# MAVROS üzerinden PX4 otopilotuna komut gönderebilmek için gerekli servis tiplerini içeri aktarıyoruz:
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
# İleri düzey trigonometrik ve matematiksel işlemler (sin, cos, vb.) için Python'un yerleşik math kütüphanesini ekliyoruz.
import math
# MAVROS'tan uçağın anlık durumunu (bağlantı var mı, arm edilmiş mi, hangi modda uçuyor) okumak için State mesajını alıyoruz.
from mavros_msgs.msg import State
# Uçağa yönelim (Attitude) veya gövde dönüş hızı (Body Rate) komutları göndermek için AttitudeTarget mesaj tipini alıyoruz.
from mavros_msgs.msg import AttitudeTarget
# Uçağın motor gücünü (Thrust) ayarlamak için Thrust mesaj tipini içeri aktarıyoruz.
from mavros_msgs.msg import Thrust

# Klavye okuma ve arka plan işlemi (Thread) için gereken standart Python kütüphaneleri:
import threading
import sys
import termios
import tty


# Kendi ROS 2 düğümümüzü (Node) temsil eden sınıfı tanımlıyoruz. Node sınıfından miras alıyor.
class FixedWingPositionControl(Node):
    # Sınıfın başlatıcı (constructor) fonksiyonu. Düğüm ayağa kalktığında ilk burası çalışır.
    def __init__(self):
        # Üst sınıfın (Node) başlatıcısını çağırıyoruz ve bu ROS 2 düğümüne 'fixed_wing_controller' adını veriyoruz.
        super().__init__('fixed_wing_controller')

        self.declare_parameter('target_x', 300.0) 
        self.declare_parameter('target_y', 0.0)   
        self.declare_parameter('target_z', 200.0)  
        

        # Uçağın anlık konumunu (X, Y, Z) tutacağımız değişken. Başlangıçta veri gelmediği için None (boş) yapıyoruz.
        self.current_pose = None

        # Uçağın anlık uçuş modunu tutacağımız değişken. Başlangıçta henüz okuyamadığımız için "BILINMIYOR" diyoruz.
        self.current_mode = "BILINMIYOR"

        # Manevra durum kontrolü
        # Uçağın an itibariyle takla manevrası yapıp yapmadığını tutan mantıksal (boolean) bayrak (flag). Başlangıçta False.
        self.is_maneuvering = False
        self.target_roll_rad = 0.0 

        # __init__ içine şu değişkenleri ekle
        # Uçağın anlık yönelimini 4 boyutlu Quaternion (qx, qy, qz, qw) cinsinden tutacak değişken.
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

        # Hızlı akan sensör verileri (pozisyon gibi) için özel bir QoS (Hizmet Kalitesi) profili oluşturuyoruz.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # Mesaj kaybolabilir, yeniden gönderme yok
            durability=DurabilityPolicy.VOLATILE,         # Geç bağlanan subscriber eski mesajları almaz
            history=HistoryPolicy.KEEP_LAST,              # Sadece son N mesajı sakla
            depth=1                                       # N=1, sadece en son mesaj
        )

        # Uçağın hedeflenen konumunu (Setpoint) MAVROS'a göndermek için bir yayıncı (Publisher) oluşturuyoruz. QoS profilimizi kullanıyor.
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # Tek mesaj, tek topik! Senkronizasyon derdi yok.
        # Uçağın hedeflenen yönelimini (Attitude) veya dönüş hızlarını göndermek için başka bir yayıncı oluşturuyoruz.
        self.att_pub = self.create_publisher(
            AttitudeTarget, 
            '/mavros/setpoint_raw/attitude', 
            10
        )

        # PX4 motorlarını çalıştırmak, mod değiştirmek ve parametre ayarlamak için istemciler.
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 

        # Uçağın o anki gerçek konumunu (Local Pose) okumak için abone oluyoruz.
        self.local_pos_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile
        )

        # SAAT: Her 1.0 saniyede bir 'log_position' görevini tetikler
        self.log_timer = self.create_timer(1.0, self.log_position)
        
        # --- YENİ BÖLÜM: KLAVYE DİNLEME ---
        # Klavye okuma işlemini ana döngüyü (50Hz publish) kilitlememesi için ayrı bir 'thread' (kanal) üzerinden başlatıyoruz.
        self.keyboard_thread = threading.Thread(target=self.wait_for_key)
        # Program kapandığında (Ctrl+C) bu thread'in de otomatik kapanması için 'daemon' bayrağını True yapıyoruz.
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Tüm tanımlamalar bittikten sonra uçuş hazırlıklarını başlatan kendi yazdığımız fonksiyonu çağırıyoruz.
        self.init_sequence()

    # Klavye tuşlarını yakalayan özel fonksiyon (Linux/Ubuntu Terminali için)
    def wait_for_key(self):
        # Terminalin o anki varsayılan ayarlarını saklıyoruz ki çıkarken bozmayalım.
        settings = termios.tcgetattr(sys.stdin)
        try:
            while True:
                # Terminali "raw" (ham) moda alıyoruz. Böylece kullanıcı tuşa basıp "Enter"a basmasını beklemeden tuşu anında okuruz.
                tty.setraw(sys.stdin.fileno())
                # Klavyeden tek bir (1) karakter okuyoruz.
                key = sys.stdin.read(1)
                
                # Eğer basılan tuş 'k' veya 'K' ise:
                if key.lower() == 'k':
                    # GÜVENLİK KİLİDİ 1: Sensörden konum verisi gelmiş mi VE irtifa 150 metreden büyük mü?
                    if self.current_pose is not None and self.current_pose.z > 150.0:
                        # GÜVENLİK KİLİDİ 2: Sadece uçağın modu OFFBOARD ise manevraya izin veriyoruz.
                        if self.current_mode == "OFFBOARD":
                            self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: İRTİFA YETERLİ, TAKLA BAŞLIYOR !!!')
                            self.start_maneuver()
                        else:
                            self.get_logger().warn('Uyarı: Manevra için uçağın OFFBOARD modunda olması gerekiyor.')
                    else:
                        # İrtifa kurtarmıyorsa veya daha uçak havalanmadıysa uyarı mesajı ver ve taklayı reddet.
                        mevcut_irtifa = self.current_pose.z if self.current_pose is not None else 0.0
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa 150 metreden düşük! (Mevcut: {mevcut_irtifa:.1f}m). Takla komutu YALITILDI.')
                
                # Eğer kullanıcı acil çıkış (q) yapmak isterse döngüyü kırabilir.
                if key.lower() == 'q':
                    break
        finally:
            # İşlem bittiğinde terminal ayarlarını tekrar eski normal haline (satır satır okuma) getiriyoruz.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def init_sequence(self):
        self.get_logger().info('Konum Kontrolu baslatiliyor... TAKLA İÇİN "K" TUŞUNA BASIN (İrtifa > 150m ise çalışır).')
        
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
        self.get_logger().info('OFFBOARD moduna geçiliyor, hedefe gidiliyor... (Takla atmak için terminalden "k" tuşuna basabilirsiniz)')
        req = SetMode.Request(custom_mode='OFFBOARD')
        self.mode_client.call_async(req)
        self.timer_offboard.cancel()


    def start_maneuver(self):
        if self.is_maneuvering:
            return
            
        self.get_logger().info('OFFBOARD: Gövde Hızı (Body Rate) ile saf takla başlatılıyor!')
        self.is_maneuvering = True
        
        # Takla süresini 5 saniye yapıyoruz.
        self.timer_stop_maneuver = self.create_timer(5.0, self.stop_maneuver)

    def stop_maneuver(self):
        self.get_logger().info('Takla bitti, normal konum (Position) hedeflerine dönülüyor.')
        self.is_maneuvering = False
        self.timer_stop_maneuver.cancel()
        
        # Sonraki olası manevralar için açıyı sıfırlıyoruz
        self.target_roll_rad = 0.0

    def publish_control_commands(self):
        if self.is_maneuvering:
            msg = AttitudeTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            # YENİ SİHİRLİ MASKE: 192 (128 Attitude Yoksay + 64 Thrust Yoksay)
            # Bu maske otopilota: "Açıları boşver, sadece sana verdiğim HIZDA kendi ekseninde dön!" der.
            msg.type_mask = 192 
            

            msg.body_rate.x = 4.0 
            msg.body_rate.y = 0.08  # Burnunu sabit tut
            msg.body_rate.z = 0.0  # Sağa sola sapma
            
            # Thrust maskelendiği için uçak taklaya girdiği anki motor gazını koruyacak
            msg.thrust = 0.0 
            
            self.att_pub.publish(msg)
            
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