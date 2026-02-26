import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        # 1. Declare the parameter with a default value
        self.declare_parameter('subscribe_topic', '/camera/csi_image')
        
        # 2. Get the parameter value
        topic_name = self.get_parameter('subscribe_topic').get_parameter_value().string_value

        self.group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10,
            callback_group=self.group)
        
        self.lane_pub = self.create_publisher(
            Float32MultiArray, '/lane_lines', 10)
        
        self.br = CvBridge()
        self.get_logger().info('Lane Detector Node has been started.')

    def lane_average(self, image, lines):
        left_fits = []
        right_fits = []
        
        if lines is None: return None

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2: continue # Evitar división por cero
            
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope, intersect = parameters[0], parameters[1]
            
            # negative slope = left lane (en coordenadas de imagen)
            # positive slope = right lane
            if slope < 0:
                left_fits.append((slope, intersect))
            else:
                right_fits.append((slope, intersect))
                
        # Average
        left_avg = np.average(left_fits, axis=0) if left_fits else None
        right_avg = np.average(right_fits, axis=0) if right_fits else None
        
        return [self.point_generator(image, left_avg), self.point_generator(image, right_avg)]

    def point_generator(self, image, fit):
        if fit is None: return None
        m, b = fit
        y1 = image.shape[0] # Fondo de la imagen
        y2 = int(y1 * 0.56)   # Look-ahead (56% de la imagen)
        x1 = int((y1 - b) / m)
        x2 = int((y2 - b) / m)
        return [x1, y1, x2, y2]

    def listener_callback(self, data):
        # self.get_logger().info('Receiving video frame') # Uncomment for debugging
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        
        if current_frame is not None:
            try: 
                # --- Image Processing Logic ---
                gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5, 5), 0)
                edges = cv2.Canny(blur, 50, 150)
                
                # ROI
                mask = np.zeros_like(edges)
                height, width = edges.shape
                poligon = np.array([[
                    (0, height), 
                    (102, 115),
                    (306, 115), 
                    (width, height)
                ]], np.int32)
                cv2.fillPoly(mask, poligon, 255)
                masked_edges = cv2.bitwise_and(edges, mask)

                # Hough
                lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=50, 
                        minLineLength=100, maxLineGap=50)
                
                # Extrapolation
                left_line, right_line = self.lane_average(current_frame, lines)
                line_image = np.copy(current_frame)
                # if lines is not None:
                #     for line in lines:
                #         x1, y1, x2, y2 = line[0]
                #         cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
                if left_line is not None:
                    # Dibujar línea AZUL (BGR) para izquierda
                    cv2.line(line_image, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (255, 0, 0), 8)
                    
                if right_line is not None:
                    # Dibujar línea ROJA (BGR) para derecha
                    cv2.line(line_image, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 0, 255), 8)

                msg_lines = Float32MultiArray()
                # Convertimos a float y usamos -1.0 si es None
                left_data = [float(x) for x in left_line] if left_line is not None else [-1.0]*4
                right_data = [float(x) for x in right_line] if right_line is not None else [-1.0]*4
                msg_lines.data = left_data + right_data
                self.lane_pub.publish(msg_lines)

                # Display the processed frame
                cv2.imshow("Detections", line_image)
                cv2.waitKey(1)
            except:
                return

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(lane_detector)

    try:
        rclpy.spin(lane_detector)
    except KeyboardInterrupt:
        pass
    finally:
        lane_detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()