import rclpy
from rclpy.node import Node
import pickle
from multiprocessing import shared_memory
import time

def shm_name_from_topic(topic_name: str) -> str:
    name = topic_name.lstrip('/')
    name = name.replace('/', '_')
    return f"shm_{name}"


def wait_shared_memory(shm_name: str, timeout: float = 60.0, interval: float = 0.1):
    deadline = time.monotonic() + timeout

    while True:
        try:
            return shared_memory.SharedMemory(name=shm_name)
        except FileNotFoundError:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"SharedMemory {shm_name} not created within {timeout} seconds")
            time.sleep(interval)


def topic_to_name(topic: str) -> str:

    if not topic:
        return ""

    # 양쪽 공백/따옴표 제거
    s = topic.strip().strip('"').strip("'")

    # 앞의 '/'들 제거
    s = s.lstrip('/')

    if not s:
        return ""

    parts = s.split('/')
    last = parts[-1]

    # 1) /vehicle/status/... -> rx_ 마지막 이름
    if len(parts) >= 3 and parts[0] == "vehicle" and parts[1] == "status":
        return f"rx_{last}"

    # 2) /control/... -> 마지막 이름만
    if len(parts) >= 2 and parts[0] == "control":
        return last

    # 3) 그 외는 그냥 마지막 토큰만
    return last



class SendNode(Node):
    def __init__(self, topic_name, topic_type,SHM_SIZE = 1024):
        self.SHM_SIZE=SHM_SIZE
        nodename=topic_to_name(topic_name)
        super().__init__(nodename)
        shm_name =shm_name_from_topic(topic_name)

        try:
            self.shm = wait_shared_memory(shm_name)
        except FileNotFoundError:
            raise RuntimeError(f"SharedMemory {shm_name} not found") from None

        self.pub = self.create_publisher(topic_type, topic_name, 10)
        self.timer = self.create_timer(0.01, self.timer_cb)  # 100Hz 확인

    def timer_cb(self):
        buf = bytes(self.shm.buf[:])
        try:
            msg = pickle.loads(buf)
            self.pub.publish(msg)
        except Exception:
            pass  

    def close(self):
        self.shm.close()

    def destroy_node(self):
        self.close()
        super().destroy_node()

class ReceiveNode(Node):
    def __init__(self, topic_name, topic_type,SHM_SIZE = 1024):
        self.SHM_SIZE=SHM_SIZE
        nodename=topic_to_name(topic_name)
        super().__init__(nodename)
        shm_name =shm_name_from_topic(topic_name)
        try:
            self.shm = shared_memory.SharedMemory(name=shm_name, create=True, size=self.SHM_SIZE)
        except FileExistsError:
            self.shm = shared_memory.SharedMemory(name=shm_name)
        self.create_subscription(topic_type, topic_name, self.rxcb, 10)


    def rxcb(self, msg):
        data = pickle.dumps(msg)
        if len(data) > self.SHM_SIZE:
            self.get_logger().warn("Message too large for shared memory!")
            return
        self.shm.buf[:len(data)] = data
    
    def close(self):
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass

    def destroy_node(self):
        self.close()
        super().destroy_node()


def CreateNode(topic_name, topic_type, node_type,SHM_SIZE = 1024):
    if node_type == "receive" or node_type == "r":
        return ReceiveNode(topic_name, topic_type,SHM_SIZE)
    elif node_type == "send" or node_type == "s":
        return SendNode(topic_name, topic_type,SHM_SIZE)


