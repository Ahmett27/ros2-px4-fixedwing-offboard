# GEREKLİ ARAÇLARI (KÜTÜPHANELERİ) İÇERİ AKTARMA BÖLÜMÜ
# Python'da kullanacağımız hazır araç setlerini projemize dahil ediyoruz.

# rclpy: ROS 2'nin Python için ana kütüphanesidir. Fabrikanın ana yönetim sistemi gibi düşünebilirsin.
import rclpy

# Fabrikadaki her bir işçiye (programa) 'Node' (Düğüm) denir. Kendi işçimizi yaratmak için bu kalıbı alıyoruz.
from rclpy.node import Node

# Telsizden haberleşirken bağlantı koparsa veya yavaşlarsa ne olacağını belirleyen kurallar bütünü (Hizmet Kalitesi).
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Dronun gideceği konumu (X, Y, Z koordinatları) ve o anki zamanı taşıyan mesaj kalıbı.
from geometry_msgs.msg import PoseStamped 

# Dronun uçuş beynine (MAVROS) özel komutlar. CommandBool motorları açmak için, SetMode ise uçuş modunu (Kalkış, Otonom vb.) değiştirmek için kullanılır.
from mavros_msgs.srv import CommandBool, SetMode

# KENDİ İŞÇİMİZİ (DÜĞÜMÜMÜZÜ) TANIMLIYORUZ
# "FixedWingPositionControl" adında, 'Node' kalıbından türetilmiş kendi programımızı oluşturuyoruz.
class FixedWingPositionControl(Node):

    # __init__ : Bu program (düğüm) ilk çalıştırıldığında otomatik olarak yapılacak hazırlıklar.
    def __init__(self):
        # İşçimize (Node) sistemde görüneceği bir isim veriyoruz: 'fixed_wing_controller'
        super().__init__('fixed_wing_controller')

        # Dışarıdan değiştirilebilen ayarlar (Parametreler) tanımlıyoruz. 
        # Böylece kodu durdurup yeniden yazmadan hedefleri değiştirebiliriz.
        # Drona 300 metre ileri gitmesini söylüyoruz.
        self.declare_parameter('target_x', 300.0) 
        # Sağa veya sola sapma diyoruz (0 metre).
        self.declare_parameter('target_y', 0.0)   
        # 70 metre yükseklikte uç diyoruz.
        self.declare_parameter('target_z', 70.0)  

        # İletişim kalitesi (QoS) ayarlarını yapıyoruz.
        # Dronlar çok hızlı hareket ettiği için eski mesajların gelmesi tehlikelidir.
        # Bu ayarlar özetle şunu der: "Bana her zaman en son ve en yeni mesajı ulaştır, gerisini boşver (BEST_EFFORT ve KEEP_LAST)."
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # YAYINCI (PUBLISHER) OLUŞTURMA: Telsizden anons yapan birini işe alıyoruz.
        # '/mavros/setpoint_position/local' isimli telsiz kanalına, 'PoseStamped' tipinde konum mesajları yayınlayacak.
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # İSTEMCİ (CLIENT) OLUŞTURMA: Doğrudan telefon açıp emir veren kişileri işe alıyoruz.
        # Dronun motorlarını çalıştırmak (Arm) için bir servis istemcisi.
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        # Dronun uçuş modunu değiştirmek için bir servis istemcisi.
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ZAMANLAYICI (TIMER) OLUŞTURMA: Sürekli tekrarlanan bir alarm kuruyoruz.
        # Her 0.1 saniyede bir (yani saniyede 10 kere) 'publish_position' isimli görev çalışacak.
        # Drona saniyede 10 kez "şuraya git" demezsek dron bağlantının koptuğunu sanıp kendini güvenliğe alır.
        self.timer = self.create_timer(0.1, self.publish_position)

        # İlk kalkış işlemlerini başlatan metodu (görevi) çağırıyoruz.
        self.init_sequence()

    # İLK KALKIŞ GÖREVLERİ
    def init_sequence(self):
        # Ekrana bilgi mesajı yazdırıyoruz.
        self.get_logger().info('L1 Algoritmasi icin Konum Kontrolu baslatiliyor...')
        
        # 'arming_client' üzerinden drona telefon açıp "Motorları çalıştır (True)" diyoruz.
        self.call_service(self.arming_client, CommandBool.Request(value=True))
        
        # 'mode_client' üzerinden drona telefon açıp "Otomatik Kalkış (AUTO.TAKEOFF) moduna geç" diyoruz.
        self.call_service(self.mode_client, SetMode.Request(custom_mode='AUTO.TAKEOFF'))
        
        # Dron kalkışa geçtikten sonra 25 saniye tırmanmasını bekliyoruz.
        # 25 saniye dolduğunda 'switch_to_offboard' isimli görevi çalıştıracak bir alarm kuruyoruz.
        self.create_timer(25.0, self.switch_to_offboard)

    # BİLGİSAYAR KONTROLÜNE (OFFBOARD) GEÇİŞ GÖREVİ
    def switch_to_offboard(self):
        # Ekrana bilgi yazdırıyoruz.
        self.get_logger().info('OFFBOARD moduna geçiliyor...')
        # Uçuş modunu 'OFFBOARD' yapmak için bir istek (req) hazırlıyoruz.
        # Offboard: "Kendi kumandanı/beynini bırak, bilgisayardan gelen X,Y,Z komutlarını dinle" modudur.
        req = SetMode.Request(custom_mode='OFFBOARD')
        # Bu isteği drona gönderiyoruz.
        self.mode_client.call_async(req)

    # KONUMU SÜREKLİ TELSİZDEN YAYINLAMA GÖREVİ (Saniyede 10 kez çalışır)
    def publish_position(self):
        # Boş bir mesaj paketi oluşturuyoruz.
        msg = PoseStamped()
        
        # Mesajın içine o anki sistem saatini ekliyoruz (Gecikmeleri tespit etmek için önemlidir).
        msg.header.stamp = self.get_clock().now().to_msg()
        # Referans çerçevemizi 'map' (harita/yerdeki sabit başlangıç noktası) olarak belirliyoruz.
        msg.header.frame_id = 'map'

        # Yukarıda 'declare_parameter' ile tanımladığımız hedefleri okuyup mesajın içine yerleştiriyoruz.
        msg.pose.position.x = self.get_parameter('target_x').value
        msg.pose.position.y = self.get_parameter('target_y').value
        msg.pose.position.z = self.get_parameter('target_z').value

        # İçini doldurduğumuz mesaj paketini radyodan (topic) yayınlıyoruz.
        self.pos_pub.publish(msg)

    # TELEFON AÇMA (SERVİS ÇAĞIRMA) YARDIMCI GÖREVİ
    # Bu görev, dronun servisi (telefon hattı) açık mı diye kontrol eder.
    def call_service(self, client, request):
        # Servis cevap verene kadar 1 saniyelik aralıklarla bekler.
        while not client.wait_for_service(timeout_sec=1.0):
            # Beklerken ekrana uyarı yazar.
            self.get_logger().info('Servis bekleniyor...')
        # Servis hazır olduğunda komutu drona gönderir.
        client.call_async(request)

# ANA PROGRAMI BAŞLATMA BÖLÜMÜ
# Python'da her şeyin başladığı yer burasıdır.
def main(args=None):
    # ROS 2 sistemini başlatıyoruz (Fabrikanın şalterini kaldırıyoruz).
    rclpy.init(args=args)
    
    # Yukarıda tanımladığımız kendi programımızdan (işçimizden) bir tane üretiyoruz.
    node = FixedWingPositionControl()
    
    # rclpy.spin: Programın kapanmasını engeller, onu sonsuz bir döngüde çalışır halde tutar. 
    # (Zamanlayıcıların ve dinleyicilerin sürekli çalışmasını sağlar).
    rclpy.spin(node)
    
    # Eğer kullanıcı programı durdurursa (örn: CTRL+C yaparsa), işçiyi sistemden temizliyoruz.
    node.destroy_node()
    # ROS 2 sistemini kapatıyoruz (Şalteri indiriyoruz).
    rclpy.shutdown()

# Bu dosya başka bir yerden çağrılmadıysa, doğrudan çalıştırıldıysa 'main' fonksiyonunu başlat.
if __name__ == '__main__':
    main()