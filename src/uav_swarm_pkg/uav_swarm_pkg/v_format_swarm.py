import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
import math

class VFormationSwarmControl(Node):
    def __init__(self):
        super().__init__('v_formation_swarm_controller')

        self.num_uavs = 5
        self.flight_phase = 'TAKEOFF_INIT'
        self.takeoff_attempts = 0
        
        # Hedef Rotası (Havalandıktan sonra gidecekleri yer)
        self.waypoints = [[1000.0, 0.0, 60.0], [1000.0, 1000.0, 60.0]]
        self.current_wp_index = 0
        self.current_poses = {i: None for i in range(1, self.num_uavs + 1)}

        # V Formasyonu Offsetleri
        self.offsets = {
            1: (0.0, 0.0, 0.0),      
            2: (-30.0, 30.0, 0.0),   
            3: (-30.0, -30.0, 0.0),  
            4: (-60.0, 60.0, 0.0),   
            5: (-60.0, -60.0, 0.0)   
        }

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pos_pubs = {}
        self.pose_subs = {}
        self.arming_clients = {}
        self.mode_clients = {}
        self.param_clients = {}

        for i in range(1, self.num_uavs + 1):
            ns = f'/uav{i}'
            # Senin sistemindeki gerçek servis yolları
            self.pos_pubs[i] = self.create_publisher(PoseStamped, f'{ns}/setpoint_position/local', qos_profile)
            self.pose_subs[i] = self.create_subscription(PoseStamped, f'{ns}/local_position/pose', 
                lambda msg, uav_id=i: self.pose_callback(msg, uav_id), qos_profile)
            
            self.arming_clients[i] = self.create_client(CommandBool, f'{ns}/cmd/arming')
            self.mode_clients[i] = self.create_client(SetMode, f'{ns}/set_mode')
            self.param_clients[i] = self.create_client(ParamSet, f'{ns}/param/set')

        self.timer = self.create_timer(0.1, self.control_loop)
        self.phase_timer = self.create_timer(1.0, self.phase_manager)

    def pose_callback(self, msg, uav_id):
        self.current_poses[uav_id] = msg.pose.position

    def set_param(self, uav_id, param, val):
        if self.param_clients[uav_id].service_is_ready():
            req = ParamSet.Request()
            req.param_id = param
            req.value.integer = val
            self.param_clients[uav_id].call_async(req)

    def phase_manager(self):
        # AŞAMA 1: KALKIŞ HAZIRLIĞI VE ATEŞLEME
        if self.flight_phase == 'TAKEOFF_INIT':
            self.takeoff_attempts += 1
            for i in range(1, self.num_uavs + 1):
                # Önce Failsafe'leri kapat (Uçakların naz yapmasını engelle)
                self.set_param(i, 'NAV_RCL_ACT', 0)
                self.set_param(i, 'COM_RCL_EXCEPT', 4)
                self.set_param(i, 'RWTO_TKOFF', 1) # Pist kalkışını zorla

                # SIFIRDAN KALKIŞ: Önce AUTO.TAKEOFF modu, sonra ARM!
                if self.mode_clients[i].service_is_ready():
                    self.mode_clients[i].call_async(SetMode.Request(custom_mode='AUTO.TAKEOFF'))
                
                if self.arming_clients[i].service_is_ready():
                    self.arming_clients[i].call_async(CommandBool.Request(value=True))

            self.get_logger().info(f'[{self.takeoff_attempts}] Pistten kalkış emri (AUTO.TAKEOFF) gönderildi...')
            
            # Lider uçağın yükselip yükselmediğini kontrol et
            leader_pose = self.current_poses[1]
            if leader_pose and leader_pose.z > 5.0:
                self.get_logger().info('!!! UÇAKLAR PİSTTEN TEKER KESTİ, TIRMANILIYOR !!!')
                self.flight_phase = 'CLIMBING'

        # AŞAMA 2: HAVADA MOD DEĞİŞİMİ
        elif self.flight_phase == 'CLIMBING':
            leader_pose = self.current_poses[1]
            if leader_pose and leader_pose.z > 30.0:
                self.get_logger().info('!!! 30 METRE GEÇİLDİ. OFFBOARD VE V FORMASYONU BAŞLIYOR !!!')
                for i in range(1, self.num_uavs + 1):
                    self.mode_clients[i].call_async(SetMode.Request(custom_mode='OFFBOARD'))
                self.flight_phase = 'MISSION'
            else:
                z = leader_pose.z if leader_pose else 0.0
                self.get_logger().info(f'Tırmanma devam ediyor... İrtifa: {z:.1f}m')

    def control_loop(self):
        # Sadece havaya çıktıktan sonra OFFBOARD setpointlerini gönder
        if self.flight_phase == 'MISSION':
            leader_target = self.waypoints[self.current_wp_index]
            for i in range(1, self.num_uavs + 1):
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.position.x = leader_target[0] + self.offsets[i][0]
                msg.pose.position.y = leader_target[1] + self.offsets[i][1]
                msg.pose.position.z = leader_target[2] + self.offsets[i][2]
                self.pos_pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VFormationSwarmControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()