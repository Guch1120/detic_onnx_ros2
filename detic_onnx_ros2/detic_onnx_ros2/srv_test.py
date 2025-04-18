# sys.argvを使用して，リクエストのコマンドライン入力引数にアクセスするためにインポート
import sys

from detic_onnx_ros2_msg.srv import GraspFeedback,ObjectDetection
import rclpy
from rclpy.node import Node
import cv2
import PIL.Image
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import os 
import time
import sklearn 
from sklearn.decomposition import PCA
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf2_ros
import transforms3d
import tf_transformations

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # コンストラクター定義は，service nodeと同じタイプと名前のclient nodeを作成する．
        # タイプと名前は，clientとserviceが通信できるように一致する必要がある．
        #self.cli = self.create_client(GraspFeedback, 'detic_result/grasp_feedback')
        self.cli = self.create_client(ObjectDetection, 'detic_result/object_feedback')
        # clientのタイプと名前に一致するserviceが利用可能かどうか，1秒に1回チェック．
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # リクエスト定義
        self.req = ObjectDetection.Request()
        self.bridge = CvBridge()
        self.broadcaster = TransformBroadcaster(self)
        self.TFpublisher = self.create_publisher(TransformStamped, 'bag_pose', 10)

    def send_request(self):
        self.req.target_name = "box"
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=100.0)
        if self.future.done():
            if self.future.result() is not None:
                response = self.future.result()
                if response.is_success:
                    #点群処理
                    #ここをFeedbackGrasp相当のノードとし、ここからdeticに処理を投げ、点群の計算や逆運動学の計算をここで行う
                    cv2.imshow("detic", self.bridge.imgmsg_to_cv2(response.masked_rgb_image,desired_encoding="bgr8"))
                    cv2.waitKey(1000)
                    cv2.destroyAllWindows()
                    #点群の取得
                    depth = self.bridge.imgmsg_to_cv2(response.masked_depth_image,desired_encoding="passthrough")
                    depth = depth.astype(np.uint16)
                    depth = o3d.geometry.Image(depth) 
                    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
                    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                    o3d.visualization.draw_geometries([pcd])
                    #クラスタ検出
                    pcd = np.asarray(pcd.points)
                    success, xyz = self.extract_biggest_cluster(pcd)


                    #主成分分析
                    pca = PCA()
                    pca.fit(xyz[:, :2])
                    center = pca.mean_
                    vec = pca.components_[0]
                    z_mean = np.mean(xyz[:, 2])
                    z_max = np.max(xyz[:, 2])
                    print(center, vec, z_mean, z_max)


                    # tf publish
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = "bag_frame"
                    t.child_frame_id = "pan_link"
                    t.transform.translation.x = center[0]
                    t.transform.translation.y = center[1]
                    t.transform.translation.z = z_mean
                    q = tf_transformations.quaternion_from_euler(0, 0, np.arctan2(vec[1], vec[0]))
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    self.TFpublisher.publish(t)
                    self.broadcaster.sendTransform(t) 
                    print(center[0], center[1], z_mean)



                    return True, xyz, center, vec, z_mean, z_max
                

                else:
                    self.get_logger().info('Request failed')
                    return False, None, None, None, None, None
                
        response =False
        return response
    
    def extract_biggest_cluster(self, pcd, eps=0.01, min_samples=10):
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(pcd)
        count_labels = np.bincount(labels[labels >= 0])
        biggest_cluster_label = np.argmax(count_labels)
        biggest_cluster = pcd[labels == biggest_cluster_label]
        return True, biggest_cluster

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    rclpy.spin_once(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
