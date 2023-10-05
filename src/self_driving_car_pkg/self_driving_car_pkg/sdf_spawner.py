import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class SDFSpawner(Node):

    def __init__(self):
        super().__init__('sdf_spawner_node')

        # Wait for the SpawnEntity service to become available
        self.client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "spawn_entity" not available, waiting...')
        self.get_logger().info("Connected to the spawner service")

    def spawn_sdf_entity(self, entity_name, sdf_model):
        request = SpawnEntity.Request()
        request.name = entity_name
        request.xml = sdf_model

        # Send the service request
        self.get_logger().info("Sending service request to /spawn_entity")
        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned entity: {entity_name}')
        else:
            self.get_logger().error(f'Failed to spawn entity: {entity_name}')
            self.get_logger().error(future.status_message)

        return future

def main():
    # Parse command-line arguments
    if len(sys.argv) < 3:
        print("Usage: sdf_spawner.py <SDF_file_path> <entity_name>")
        return

    sdf_path = sys.argv[1]
    entity_name = sys.argv[2]
    sdf_model_xml = open(sdf_path, 'r').read()

    rclpy.init()

    # Async call to spawn the SDF entity
    spawner_node = SDFSpawner()
    spawner_node.spawn_sdf_entity(entity_name, sdf_model_xml)
    spawner_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
