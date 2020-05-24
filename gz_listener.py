#Navid Kayhani (navidk381) | May 20,2020

'''
This script listens to /gazebo/default/pose/info and prints it out on the screen
'''
import trollius as asyncio
from trollius import From
import pygazebo
import pygazebo.msg.poses_stamped_pb2
# https://github.com/tensorflow/tensorflow/issues/890
# https://github.com/jpieper/pygazebo
# https://stackoverflow.com/questions/43948454/python-invalid-syntax-with-async-def

class GazeboMessageSubscriber: 

    def __init__(self, timeout=30):
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout

    @asyncio.coroutine
    def connect(self):
        connected = False
        for i in range(self.timeout):
            try:
                self.manager = yield From(pygazebo.connect(('localhost', 11345)))
                connected = True
                break
            except Exception as e:
                pass
            yield From(asyncio.sleep(1))

        if connected: 
            self.poses_subscriber = self.manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self.poses_callback)

            yield From(self.poses_subscriber.wait_for_connection())
            self.running = True
            while self.running:
                yield From(asyncio.sleep(0.1))
        else:
            raise Exception("Timeout connecting to Gazebo.")

    def poses_callback(self, data):
        # message = pygazebo.msg.gz_string_pb2.GzString.FromString(data)
        # print('Received message:', message.data)
        self.poses_stamped = pygazebo.msg.poses_stamped_pb2.PosesStamped() #check the msg for more detail (poses_stamped_pb2.py)
        s = self.poses_stamped.FromString(data)
        #for more information about the parsed Gazebo messages, please see
        #<your_ros_ws>/gz_listener/src/py3gazebo_x/pygazebo/msg/poses_stamped_pb2.py
        print(s.pose[0])


def main():
    gztest = GazeboMessageSubscriber()
    gztest.loop.run_until_complete(gztest.connect())

if __name__ == "__main__":
    main()
        