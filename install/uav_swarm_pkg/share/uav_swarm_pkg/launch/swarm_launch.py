from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Uçak (uav1) için düğüm
        Node(
            package='uav_swarm_pkg', # Kendi paket adını yazmalısın
            executable='suru_kontrol', # setup.py'da tanımladığın executable adı
            name='uav1_controller',
            parameters=[{
                'target_x': 300.0,
                'target_y': 10.0,  # uav1 biraz solda
                'target_z': 70.0,
                'drone_id': 'uav1' # Kodun içinde kullanacağımız ID
            }]
        ),
        
        # 2. Uçak (uav2) için düğüm
        Node(
            package='uav_swarm_pkg', 
            executable='suru_kontrol',
            name='uav2_controller',
            parameters=[{
                'target_x': 300.0,
                'target_y': -10.0, # uav2 biraz sağda (çarpışmamaları için)
                'target_z': 70.0,
                'drone_id': 'uav2'
            }]
        )
    ])