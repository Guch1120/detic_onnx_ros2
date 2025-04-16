# sys.argvを使用して，リクエストのコマンドライン入力引数にアクセスするためにインポート
import sys

from detic_onnx_ros2_msg.srv import GraspFeedback,ObjectDetection
import rclpy
from rclpy.node import Node


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

    def send_request(self):
        self.req.target_name = "aaaaaaaa"
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        # ループは，futureシステムが実行されている限り，serviceからの応答があるかどうかを確認．
        # serviceが応答を送信した場合，結果をログメッセージに残す．
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    str(response.is_success))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
