# ROS 2'nin Python dilindeki ana iletişim kütüphanesini içe aktarıyoruz.
import rclpy
# rclpy içinden, yazdığımız sınıfı bir ROS 2 düğümüne (Node) dönüştürecek temel sınıfı alıyoruz.
from rclpy.node import Node
# Sensör verileri (GPS, IMU) gibi hızlı akan veriler için haberleşme kalitesini (QoS) ayarlayacağımız sınıfları içe aktarıyoruz.
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# Uzaydaki 3 boyutlu konumu (X, Y, Z) ve yönelimi bildirmek için standart geometri mesajını içeri aktarıyoruz.
from geometry_msgs.msg import PoseStamped
# MAVROS üzerinden PX4 otopilotuna komut göndermek için gerekli servisleri içeri aktarıyoruz:
# CommandBool: İHA'nın motorlarını açıp/kapatmak (Arm/Disarm) için.
# SetMode: Uçuş modunu (AUTO.TAKEOFF, OFFBOARD vb.) değiştirmek için.
# ParamSet: Uçuş öncesi otopilotun iç parametrelerini ayarlamak için.
from mavros_msgs.srv import CommandBool, SetMode, ParamSet 
# İleri düzey trigonometri ve matematik işlemleri (sin, cos, radyan hesapları) için standart math kütüphanesi.
import math
# İHA'nın o anki bağlantı durumunu ve uçuş modunu okumak için State mesaj tipi.
from mavros_msgs.msg import State
# İHA'ya belirli bir açı veya dönüş hızı (takla atma gibi) komutları vermek için AttitudeTarget mesaj tipi.
from mavros_msgs.msg import AttitudeTarget
# İHA'nın motor itki (gaz) seviyesini kontrol etmek için Thrust mesaj tipi.
from mavros_msgs.msg import Thrust

# --- ARKA PLAN (THREAD) VE KLAVYE DİNLEME KÜTÜPHANELERİ ---
# Eşzamanlı işlem (aynı anda hem uçuş döngüsü hem de klavye okuma) yapabilmek için threading kütüphanesi.
import threading
# Sistemin standart girdi/çıktı (klavye) akışlarına erişmek için sys kütüphanesi.
import sys
# Linux/Ubuntu tabanlı sistemlerde terminalin davranışını (tuş basımını algılama) değiştirmek için termios kütüphanesi.
import termios
# Terminali 'raw' (ham) moda alıp, Enter tuşuna basılmasını beklemeden tuşları okumak için tty kütüphanesi.
import tty


# ROS 2 Node (Düğüm) sınıfından miras alarak kendi uçuş kontrolcümüzü tanımlıyoruz.
class FixedWingPositionControl(Node):
    # Sınıftan bir nesne yaratıldığında ilk çalışacak olan başlatıcı (constructor) fonksiyon.
    def __init__(self):
        # Miras aldığımız Node sınıfını başlatıyoruz ve ROS 2 ağında bu düğümün adını 'fixed_wing_controller' yapıyoruz.
        super().__init__('fixed_wing_controller')

        # Düğümümüz için dışarıdan terminal ile değiştirilebilir (dinamik) parametreler tanımlıyoruz.
        # İHA'nın gideceği hedef X koordinatı. (Başlangıca göre 300 metre ileri)
        self.declare_parameter('target_x', 300.0) 
        # İHA'nın gideceği hedef Y koordinatı. (Sağa veya sola sapma yok, 0.0)
        self.declare_parameter('target_y', 0.0)   
        # İHA'nın çıkmasını istediğimiz hedef Z (irtifa) koordinatı. (200 metre yükseklik)
        self.declare_parameter('target_z', 200.0)  
        

        # Otopilottan gelecek X, Y, Z konum bilgisini tutacağımız değişken. İlk başta henüz veri gelmediği için 'None'.
        self.current_pose = None

        # Otopilottan gelecek uçuş modunu tutacağımız değişken. Veri gelene kadar "BILINMIYOR" olarak ayarlıyoruz.
        self.current_mode = "BILINMIYOR"

        # --- MANEVRA DURUM DEĞİŞKENLERİ ---
        # İHA'nın an itibariyle takla atıp atmadığını kontrol eden mantıksal (boolean) bayrak. Başlangıçta False.
        self.is_maneuvering = False
        # Hedeflenen yatış açısını tutacak değişken (Bu kodda hız ile takla attığımız için aktif kullanılmıyor).
        self.target_roll_rad = 0.0  

        # Otopilottan gelecek uçağın uzaydaki yönelim açısını (Quaternion: x,y,z,w) tutacağımız değişken.
        self.current_quat = None
        # İleriki geliştirmeler için Yaw (Sapma) açısını kilitlemek amaçlı değişken.
        self.locked_yaw = 0.0
        # İleriki geliştirmeler için Pitch (Yunuslama) açısını kilitlemek amaçlı değişken.
        self.locked_pitch = 0.0
        
        # --- OFFBOARD ANA DÖNGÜSÜ ---
        # OFFBOARD modunun kapanmaması için uçağa çok hızlı veri göndermeliyiz. 
        # create_timer ile her 0.02 saniyede bir (50 Hz frekans) 'publish_control_commands' fonksiyonunu tetikliyoruz.
        self.timer = self.create_timer(0.02, self.publish_control_commands)
        

        # MAVROS'un '/mavros/state' topiğine abone oluyoruz. Uçağın modu değiştiğinde 'state_callback' fonksiyonu çalışacak.
        # Buradaki '10', geçmişe dönük tutulacak maksimum mesaj sayısıdır (QoS depth).
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_callback, 
            10
        )

        # Yüksek frekanslı (çok hızlı akan) sensör verileri için özel bir İletişim Kalitesi (QoS) profili yaratıyoruz.
        qos_profile = QoSProfile(
            # BEST_EFFORT: Hızlı iletişim. Mesaj kaybolursa tekrar göndermekle vakit kaybetme (TCP yerine UDP mantığı).
            reliability=ReliabilityPolicy.BEST_EFFORT,   
            # VOLATILE: Mesajlar kalıcı değildir. Sonradan bağlanan biri eski sensör verilerini görmez.
            durability=DurabilityPolicy.VOLATILE,         
            # KEEP_LAST: Hafızada sadece gelen son N adet mesajı tut.
            history=HistoryPolicy.KEEP_LAST,              
            # N=1: Sadece en taze (en son gelen) mesajı tut, öncekileri çöpe at.
            depth=1                                       
        )

        # Uçağa gitmesi gereken X, Y, Z konumlarını bildireceğimiz yayıncı (Publisher) nesnesini oluşturuyoruz.
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # Uçağa yönelim (açı) veya dönüş hızı (rate) bildireceğimiz yayıncı nesnesini oluşturuyoruz.
        self.att_pub = self.create_publisher(
            AttitudeTarget, 
            '/mavros/setpoint_raw/attitude', 
            10
        )

        # PX4 ile iletişim kuracak ROS Servis İstemcilerini (Client) tanımlıyoruz.
        # Motorları arm etmek (çalıştırmak) için istemci.
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        # Mod değiştirmek (AUTO, OFFBOARD) için istemci.
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        # Parametre ayarlarını değiştirmek için istemci.
        self.param_client = self.create_client(ParamSet, '/mavros/param/set') 

        # Uçağın şu anki X, Y, Z konumunu veren 'local_position/pose' topiğine abone oluyoruz.
        # Veri her geldiğinde 'pose_callback' fonksiyonu otomatik olarak çalışacak.
        self.local_pos_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile
        )

        # Terminal ekranına sürekli bilgi yazdırıp kirlilik yaratmamak için saniyede sadece 1 kez çalışan log zamanlayıcısı.
        self.log_timer = self.create_timer(1.0, self.log_position)
        
        # --- KLAVYE DİNLEME İŞ PARÇACIĞI (THREAD) ---
        # ROS 2'nin kendi döngüsünü (spin) kilitlememek (bloklamamak) için klavye dinleme işlemini tamamen ayrı bir kanalda açıyoruz.
        # target=self.wait_for_key diyerek çalışacak fonksiyonu gösteriyoruz.
        self.keyboard_thread = threading.Thread(target=self.wait_for_key)
        # Program (Node) kapatıldığında, arka planda çalışan bu klavye dinleyicisinin de otomatik kapanması için 'daemon' yapıyoruz.
        self.keyboard_thread.daemon = True
        # Ayrı kanaldaki klavye dinleme işlemini resmi olarak başlatıyoruz.
        self.keyboard_thread.start()
        
        # init fonksiyonu bitmeden, uçuş öncesi hazırlıkları yapacak olan sıralamayı başlatıyoruz.
        self.init_sequence()

    # --- KLAVYE OKUMA VE GÜVENLİK KONTROL FONKSİYONU ---
    def wait_for_key(self):
        # Terminalin o anki geçerli ayarlarını (örneğin yazılanı ekranda gösterme, enter bekleme) bir değişkene kopyalıyoruz.
        settings = termios.tcgetattr(sys.stdin)
        try:
            # Sonsuz bir döngü başlatıyoruz, program açık kaldığı sürece klavyeyi dinleyecek.
            while True:
                # sys.stdin (Standart Girdi - Klavye) için terminali 'raw' moda alıyoruz. Enter'a basmaya gerek kalmadan tuş okunur.
                tty.setraw(sys.stdin.fileno())
                # Klavyeden sadece 1 karakter uzunluğunda veri okuyoruz.
                key = sys.stdin.read(1)
                
                # Okunan tuşu küçülterek 'k' harfi olup olmadığını kontrol ediyoruz.
                if key.lower() == 'k':
                    # --- GÜVENLİK KİLİDİ 1 ---
                    # 'current_pose' None değilse (yani sensörler çalışıyor ve konum geliyorsa) 
                    # VE uçağın Z (irtifa) değeri 150.0 metreden kesinlikle büyükse taklaya izin verme sürecini başlat.
                    if self.current_pose is not None and self.current_pose.z > 150.0:
                        # --- GÜVENLİK KİLİDİ 2 ---
                        # Uçağın o anki modu 'OFFBOARD' (Dış bilgisayar kontrolü) ise manevrayı tetikle.
                        if self.current_mode == "OFFBOARD":
                            # Terminale her şeyin yolunda olduğunu ve taklanın başladığını yazdır.
                            self.get_logger().info('!!! KLAVYEDEN TETİKLENDİ: İRTİFA YETERLİ, TAKLA BAŞLIYOR !!!')
                            # Manevrayı başlatan fonksiyonu çağır.
                            self.start_maneuver()
                        else:
                            # Uçak OFFBOARD modunda değilse (örneğin hala kalkış yapıyorsa) uyarı ver.
                            self.get_logger().warn('Uyarı: Manevra için uçağın OFFBOARD modunda olması gerekiyor.')
                    else:
                        # Eğer irtifa 150'den düşükse veya veri yoksa, Python'un tek satırlık if (ternary) yapısı ile 
                        # mevcut irtifayı alıyoruz (veri yoksa 0.0 kabul et).
                        mevcut_irtifa = self.current_pose.z if self.current_pose is not None else 0.0
                        # Pilotu uyar ve takla komutunu iptal et (Yalıt).
                        self.get_logger().warn(f'GÜVENLİK UYARISI: İrtifa 150 metreden düşük! (Mevcut: {mevcut_irtifa:.1f}m). Takla komutu YALITILDI.')
                
                # Acil durum veya terminalden çıkmak istenirse 'q' tuşuna basıldığında döngüyü kırar.
                if key.lower() == 'q':
                    break
        finally:
            # Döngü kırıldığında veya kod kapatıldığında, terminalin bozduğumuz ayarlarını ('raw' mod) eski normal haline getiriyoruz.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # --- UÇUŞ HAZIRLIK VE KALKIŞ SIRALAMASI ---
    def init_sequence(self):
        # Sistemin başladığına dair log mesajı atıyoruz.
        self.get_logger().info('Konum Kontrolu baslatiliyor... TAKLA İÇİN "K" TUŞUNA BASIN (İrtifa > 150m ise çalışır).')
        
        # MAVROS veri bağlantısı koptuğunda uçağın acil inişe (Failsafe) geçmemesi için bu özelliği (0) kapatıyoruz.
        self.set_px4_param('NAV_DLL_ACT', 0)      
        # Kumanda radyo sinyali koptuğunda acil inişe geçmemesi için bu özelliği (0) kapatıyoruz.
        self.set_px4_param('NAV_RCL_ACT', 0)      
        # Kumanda kopsa dahi uçuş modunun (OFFBOARD) iptal olmaması için kural dışı bırakma (Except) değerini 4 yapıyoruz.
        self.set_px4_param('COM_RCL_EXCEPT', 4)   
        
        # Otopilotun doğrudan kumandadan gelen sinyaller yerine MAVROS üzerinden gelen verileri ana kumanda kabul etmesi için 1 yapıyoruz.
        self.set_px4_param('COM_RC_IN_MODE', 1) 
        
        # Motorları arm (aktif) etmek için 'arming_client' üzerinden 'True' isteği yolluyoruz.
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        # Uçağın otonom kalkış yapması için modu 'AUTO.TAKEOFF' olarak ayarlıyoruz.
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
        # Uçağın kalkışı gerçekleştirmesi ve hedeflenen minimum irtifaya ulaşması için 10 saniye süre veriyoruz.
        # 10 saniye dolduğunda 'switch_to_offboard' fonksiyonu otomatik çağrılacak.
        self.timer_offboard = self.create_timer(10.0, self.switch_to_offboard)

    # Parametre atama işlemini tek satırda halletmek için yazdığımız yardımcı (helper) fonksiyon.
    def set_px4_param(self, param_id, integer_value=0):
        # MAVROS parametre ayarlama servis isteğini oluşturuyoruz.
        req = ParamSet.Request()
        # Değişecek parametrenin ID'sini (ismini) atıyoruz.
        req.param_id = param_id
        # Parametrenin alacağı tam sayı değerini atıyoruz.
        req.value.integer = integer_value
        # Servisi asenkron (kodu bekletmeden) olarak çağırıyoruz.
        self.param_client.call_async(req)

    # Otomatik kalkıştan (10 sn sonra) ROS 2 kontrollü uçuşa geçiş fonksiyonu.
    def switch_to_offboard(self):
        # Terminale OFFBOARD geçiş bilgisini logluyoruz.
        self.get_logger().info('OFFBOARD moduna geçiliyor, hedefe gidiliyor... (Takla atmak için terminalden "k" tuşuna basabilirsiniz)')
        # OFFBOARD mod isteği yaratıyoruz.
        req = SetMode.Request(custom_mode='OFFBOARD')
        # MAVROS'a modu değiştirmesi için asenkron istek yolluyoruz.
        self.mode_client.call_async(req)
        # Sadece 1 kez çalışması gereken 10 saniyelik zamanlayıcıyı işi bittiği için iptal ediyoruz (hafıza temizliği).
        self.timer_offboard.cancel()

        # DİKKAT: Eski kodda bulunan "20 saniye sonra takla at" otomasyonu kaldırıldı.
        # Artık taklayı sadece yukarıdaki wait_for_key() fonksiyonu üzerinden 'K' tuşu tetikleyecek.

    # Takla işleminin mekaniğini hazırlayan fonksiyon.
    def start_maneuver(self):
        # Eğer İHA zaten o an takla atıyorsa (True ise) fonksiyonu hemen terk et (return), üst üste taklayı engelle.
        if self.is_maneuvering:
            return
            
        # Log mesajı atıyoruz.
        self.get_logger().info('OFFBOARD: Gövde Hızı (Body Rate) ile saf takla başlatılıyor!')
        # Ana kontrol döngüsünün tavrını değiştirmek için manevra bayrağını True yapıyoruz.
        self.is_maneuvering = True
        
        # Takla manevrasının tam 5.0 saniye sürmesini istiyoruz.
        # 5 saniye dolduğunda 'stop_maneuver' fonksiyonu çağrılacak.
        self.timer_stop_maneuver = self.create_timer(5.0, self.stop_maneuver)

    # 5 saniyelik takla süresi bittiğinde çağrılan toparlanma fonksiyonu.
    def stop_maneuver(self):
        # Log mesajı ile uçağın normal uçuşa döneceğini bildiriyoruz.
        self.get_logger().info('Takla bitti, normal konum (Position) hedeflerine dönülüyor.')
        # Bayrağı False yaparak ana döngünün tekrar X, Y, Z konum komutları yayınlamasını sağlıyoruz.
        self.is_maneuvering = False
        # Taklayı bitiren bu 5 saniyelik zamanlayıcıyı da işi bittiği için iptal ediyoruz.
        self.timer_stop_maneuver.cancel()
        
        # İleride eklenebilecek yeni manevralar için kalıntı kalmasın diye açıyı sıfırlıyoruz.
        self.target_roll_rad = 0.0

    # Saniyede 50 kere (0.02s) çalışan, otopilotu hayatta tutan ve onu yönlendiren ANA KALP ATIŞI döngüsü.
    def publish_control_commands(self):
        # Eğer klavyeden K'ye basılmış, güvenlik aşılmış ve manevra başlatılmışsa (True):
        if self.is_maneuvering:
            # Sadece hedef yönelim veya dönüş hızlarını iletebileceğimiz mesaj tipini yaratıyoruz.
            msg = AttitudeTarget()
            # MAVROS'un komutu reddetmemesi için güncel ROS zaman damgasını mesaja vuruyoruz.
            msg.header.stamp = self.get_clock().now().to_msg()
            # Koordinat sisteminin dünya değil, doğrudan uçağın gövdesi (base_link) olduğunu belirtiyoruz.
            msg.header.frame_id = 'base_link'
            
            # YENİ SİHİRLİ MASKE: 192 (İkilik sistemde 11000000)
            # 128 (Quaternion açıları yoksay) + 64 (Thrust itkisini yoksay) = 192.
            # Otopilota: "Sadece body_rate (dönüş hızı) değerlerini dikkate al" diyen çok kritik bir maskedir.
            msg.type_mask = 192 
            
            # X Ekseninde (Uçağın burnundan kuyruğuna eksen) dönüş (Roll/Yatış) hızı.
            # Saniyede 12.0 radyanlık (Çok hızlı, fırıldak gibi) bir takla emri veriyoruz.
            msg.body_rate.x = 12.0 
            # Y Ekseninde (Kanattan kanada) dönüş hızı. Uçağın burnunu düşürmemesi için hafif telafi (0.08).
            msg.body_rate.y = 0.08  
            # Z Ekseninde (Tavandan tabana) dönüş hızı. Burnunu sağa sola çevirmemesini, 0 olmasını istiyoruz.
            msg.body_rate.z = 0.0  
            
            # Maske 192 ile motor komutunu yoksaydığımız için buraya ne yazarsak yazalım otopilot bunu görmezden gelecek
            # ve uçağın taklaya girmeden önceki gaz/hız seviyesini koruyacaktır.
            msg.thrust = 0.0 
            
            # Hazırladığımız agresif takla hız komutlarını MAVROS topiğine fırlatıyoruz.
            self.att_pub.publish(msg)
            
        # Eğer manevrada değilsek (False), uçağın normal hedefine gitmesini sağlıyoruz.
        else:
            # Hedeflenen X, Y, Z pozisyonlarını barındıracak konum mesajı tipini oluşturuyoruz.
            msg = PoseStamped()
            # Mesaja yine zaman damgası basıyoruz.
            msg.header.stamp = self.get_clock().now().to_msg()
            # Çerçeveyi 'map' (harita / dünya ekseni) yapıyoruz ki uçak kendi eksenine değil dünyaya göre uçsun.
            msg.header.frame_id = 'map'
            # Node başlatılırken belirlediğimiz (X: 300) parametresini okuyup mesaja atıyoruz.
            msg.pose.position.x = self.get_parameter('target_x').value
            # Node başlatılırken belirlediğimiz (Y: 0.0) parametresini okuyup mesaja atıyoruz.
            msg.pose.position.y = self.get_parameter('target_y').value
            # Node başlatılırken belirlediğimiz (Z: 200.0) parametresini okuyup mesaja atıyoruz.
            msg.pose.position.z = self.get_parameter('target_z').value
            
            # Hazırladığımız barışçıl hedef konum komutunu MAVROS topiğine fırlatıyoruz.
            self.pos_pub.publish(msg)


    # İnsan beyninin algıladığı 3 boyutlu Euler (Roll, Pitch, Yaw) açılarını,
    # Otopilotun matematiksel olarak "Gimbal Lock" (kilitlenme) yaşamadan hesap yapmasını sağlayan 
    # 4 Boyutlu Quaternion (qx, qy, qz, qw) formatına çeviren özel trigonometrik fonksiyon.
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Matematiksel formül gereği Yaw (Sapma) açısının yarısının kosinüsü alınır.
        cy = math.cos(yaw * 0.5)
        # Yaw (Sapma) açısının yarısının sinüsü alınır.
        sy = math.sin(yaw * 0.5)
        # Pitch (Yunuslama) açısının yarısının kosinüsü alınır.
        cp = math.cos(pitch * 0.5)
        # Pitch (Yunuslama) açısının yarısının sinüsü alınır.
        sp = math.sin(pitch * 0.5)
        # Roll (Yatış) açısının yarısının kosinüsü alınır.
        cr = math.cos(roll * 0.5)
        # Roll (Yatış) açısının yarısının sinüsü alınır.
        sr = math.sin(roll * 0.5)

        # 4 Boyutlu sayının W (Gerçel/Reel) bileşeni hesaplanır.
        qw = cr * cp * cy + sr * sp * sy
        # X bileşeni hesaplanır.
        qx = sr * cp * cy - cr * sp * sy
        # Y bileşeni hesaplanır.
        qy = cr * sp * cy + sr * cp * sy
        # Z bileşeni hesaplanır.
        qz = cr * cp * sy - sr * sp * cy

        # Hesaplanan 4 değeri, ROS 2 ve MAVROS'un anlayabileceği bir dizi [array] olarak geri döndürür.
        return [qx, qy, qz, qw]
    
    
    # Otopilottan '/mavros/local_position/pose' verisi yayınlandığı an tetiklenen Callback (geri çağırım) fonksiyonu.
    def pose_callback(self, msg):
        # Gelen mesaj paketinin içindeki konum objesini (X,Y,Z) doğrudan sınıfın güncel konum değişkenine kopyalar.
        self.current_pose = msg.pose.position
        # Gelen mesajın içindeki yönelim objesini (quaternion) sınıfın güncel yönelim değişkenine kopyalar.
        self.current_quat = msg.pose.orientation

    # Otopilottan '/mavros/state' verisi yayınlandığı an tetiklenen Callback fonksiyonu.
    def state_callback(self, msg):
        # Gelen mesajdan uçağın o anki modunu (örn: "AUTO.TAKEOFF", "OFFBOARD") okur ve değişkene yazar.
        self.current_mode = msg.mode

    # 4 Boyutlu karmaşık Quaternion verisini alıp, sadece ihtiyacımız olan Pitch (Aşağı/Yukarı) ve Yaw (Sağ/Sol) Euler açılarını çıkaran fonksiyon.
    def get_pitch_yaw_from_quaternion(self, q):
        # Eğer henüz otopilottan geçerli bir açı gelmemişse çökmeyi engellemek için 0.0 döndürüp çıkar.
        if q is None:
            return 0.0, 0.0
        
        # Pitch (Yunuslama) hesabı: İlgili Quaternion çarpımlarından sinüs terimi (sinp) elde edilir.
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        # Matematikte sinüs 1'den büyük olamaz. Sensör hataları yüzünden (örn: 1.0001) olursa yazılım çöker. 
        # Bu 'if' bloğu kayan nokta (floating point) hatalarını sınırlandırır.
        if abs(sinp) >= 1:
            # İşaretine (artı/eksi) bağlı kalarak direkt maksimum açıyı (Pi/2 yani 90 derece) yapıştırır.
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            # Hata yoksa arksinüs (asin) fonksiyonuyla açının radyan karşılığını bulur.
            pitch = math.asin(sinp)
            
        # Yaw (Sapma) hesabı: Atan2 (4 bölgeli arktanjant) fonksiyonu için önce Y terimi hesaplanır.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # Atan2 fonksiyonu için X terimi hesaplanır.
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # Atan2 sayesinde yön karmaşası yaşanmadan uçağın burnunun pusulada nereye baktığı (Yaw) bulunur.
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Sadece Pitch ve Yaw açılarını geri döndürür.
        return pitch, yaw
 

    # Saniyede 1 kere uçağın durumunu yazdıran fonksiyon (Ekran kirliliğini engeller).
    def log_position(self):
        # Eğer İHA'nın konumu verisi okunabiliyorsa yazdırma işlemine gir.
        if self.current_pose is not None:
            # X, Y ve Z pozisyonlarını objenin içinden çek.
            x = self.current_pose.x
            y = self.current_pose.y
            z = self.current_pose.z
            # F-String formatlaması (.1f) kullanarak sayıları virgülden sonra tek haneye kırp ve terminale şıkça yazdır.
            self.get_logger().info(f'RADAR: X={x:.1f} | Y={y:.1f} | İrtifa={z:.1f} | MOD: {self.current_mode}')

    # ROS servislerini asenkron (programı dondurmadan) olarak çağırmak için genel yardımcı metod.
    def call_service(self, client, request):
        # Çağrılmak istenen servisin MAVROS tarafında hazır olmasını en fazla 3 saniye boyunca bekle.
        if client.wait_for_service(timeout_sec=3.0):
            # Servis açıksa ve hazırsa, komutu / isteği ilet.
            client.call_async(request)

# Terminalden program başlatıldığında devreye giren en ana giriş noktası (Main) fonksiyonu.
def main(args=None):
    # rclpy altyapısını kurarak ROS 2 ağını etkinleştirir.
    rclpy.init(args=args)
    # Yukarıda yüzlerce satır boyunca yazdığımız 'FixedWingPositionControl' sınıfını ayağa kaldırır (instance yaratır).
    node = FixedWingPositionControl()
    # ROS 2'nin spin (döndürme) mekanizması. Bu sayede yazılım kapanmaz, sonsuza kadar callback'leri ve zamanlayıcıları (timer) dinler.
    rclpy.spin(node)
    # Eğer kullanıcı terminalde Ctrl+C yapıp spin'i kırarsa, düğümü (node) hafızadan temizler ve güvenlice yok eder.
    node.destroy_node()
    # ROS 2 bağlantılarını tamamen sonlandırıp programı kapatır.
    rclpy.shutdown()

# Python dosyasının başka bir dosyaya import olarak değil de, doğrudan kendisinin çalıştırıldığını doğrulayan Python kontrol bloğu.
if __name__ == '__main__':
    # Kod doğrudan tetiklendiyse main() fonksiyonunu çağır.
    main()