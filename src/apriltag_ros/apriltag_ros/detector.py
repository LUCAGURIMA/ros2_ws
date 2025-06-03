import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point32
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from tf_transformations import quaternion_from_matrix
import message_filters

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Parâmetros
        self.declare_parameter('tag_size', 0.16)
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value
        
        # Inicializar detector (API compatível com apriltag 0.0.16)
        options = apriltag.DetectorOptions()
        options.families = 'tagStandard41h12'
        options.quad_decimate = 1.5
        options.quad_sigma = 0.0
        options.nthreads = 4
        options.refine_edges = 1
        
        self.detector = apriltag.Detector(options)
        
        # Subscribers sincronizados
        self.image_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/image_raw', 
            qos_profile=qos_profile_sensor_data
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, 
            CameraInfo, 
            '/camera_info', 
            qos_profile=qos_profile_sensor_data
        )
        
        # Sincronizador de mensagens
        self.synchronizer = message_filters.TimeSynchronizer(
            [self.image_sub, self.camera_info_sub], 
            10
        )
        self.synchronizer.registerCallback(self.image_callback)
        
        # Publisher
        self.detections_pub = self.create_publisher(
            AprilTagDetectionArray, 
            '/detections', 
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # DEBUG: Adicionar visualização
        self.create_timer(0.1, self.debug_visualization)
        self.last_debug_img = None

    def image_callback(self, image_msg, camera_info_msg):
        try:
            # Converter imagem para escala de cinza
            gray = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')
        except Exception as e:
            self.get_logger().error(f'Erro CV Bridge: {str(e)}')
            return

        # Detecção de tags
        detections = self.detector.detect(gray)
        detections_array = AprilTagDetectionArray()
        detections_array.header = image_msg.header

        # Processar detecções
        for det in detections:
            # Criar mensagem de detecção
            detection_msg = AprilTagDetection()
            detection_msg.id = det.tag_id
            detection_msg.hamming = det.hamming
            detection_msg.decision_margin = det.decision_margin
            detection_msg.centre = Point32(x=det.center[0], y=det.center[1])
            
            # Adicionar cantos (forma mais compatível)
            for corner in det.corners:
                p = Point32()
                p.x = corner[0]
                p.y = corner[1]
                detection_msg.corners.append(p)
                
            detections_array.detections.append(detection_msg)

            # Estimativa de pose
            self.estimate_pose(det, camera_info_msg, image_msg.header)

        # Publicar detecções
        self.detections_pub.publish(detections_array)
        
        # Salvar imagem para debug
        debug_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for det in detections:
            corners = det.corners.astype(int)
            cv2.polylines(debug_img, [corners], True, (0, 255, 0), 2)
            cv2.putText(debug_img, str(det.tag_id), 
                        (int(det.center[0]), int(det.center[1])), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        self.last_debug_img = debug_img

    def estimate_pose(self, detection, camera_info, header):
        # Pontos 3D do objeto (ordem corrigida)
        object_points = np.array([
            [-self.tag_size/2,  self.tag_size/2, 0],   # bottom-left
            [ self.tag_size/2,  self.tag_size/2, 0],   # bottom-right
            [ self.tag_size/2, -self.tag_size/2, 0],   # top-right
            [-self.tag_size/2, -self.tag_size/2, 0]    # top-left
        ], dtype=np.float32)
        
        # Pontos 2D da imagem
        image_points = np.array(detection.corners, dtype=np.float32)
        
        # Parâmetros da câmera
        camera_matrix = np.array(camera_info.k).reshape(3,3)
        dist_coeffs = np.array(camera_info.d)[:4] if camera_info.d and len(camera_info.d) >= 4 else np.zeros(4)
        
        # Resolver PnP
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            self.get_logger().warn(f'Falha PnP para tag {detection.tag_id}')
            return

        # Converter para transformada
        transform = TransformStamped()
        transform.header.stamp = header.stamp
        transform.header.frame_id = camera_info.header.frame_id
        transform.child_frame_id = f'tag_{detection.tag_id}'
        
        # Extração corrigida dos vetores de translação
        tvec = tvec.ravel()
        transform.transform.translation.x = tvec[0]
        transform.transform.translation.y = tvec[1]
        transform.transform.translation.z = tvec[2]
        
        # Converter rotação para quaternion
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, :3] = cv2.Rodrigues(rvec)[0]
        q = quaternion_from_matrix(rotation_matrix)
        
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        # Publicar transformada
        self.tf_broadcaster.sendTransform(transform)
    
    def debug_visualization(self):
        """Mostrar imagem com detecções para debug"""
        if self.last_debug_img is not None:
            cv2.imshow("AprilTag Detections", self.last_debug_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()