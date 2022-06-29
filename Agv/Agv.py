from time import sleep, perf_counter
from threading import Thread
import numpy as np
import uuid
import sys


sys.path.append("..")

from RabbitMq.RabbitMqSender import RabbitMqSender

class Agv:
    LOOP_TIME = 0.2

    def __init__(self, USE_RMQ = False):
        # property
        self.id = str(uuid.uuid4())
        self.pos = np.array([0,0])
        self.speed = 100.0
        self.angle = 0
        self.target = []

        # RMQ
        self.rmq = None
        if USE_RMQ:
            self.rmq = RabbitMqSender()

        # tread
        self.running = True
        self.thread = Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        while self.running:
            if len(self.target) > 0:
                v = self.target[-1] - self.pos
                if np.linalg.norm(v) < 1:
                    self.pos = self.target
                else:
                    v_hat = v / np.linalg.norm(v)
                    self.pos = self.pos + v_hat * Agv.LOOP_TIME * self.speed

            if self.rmq is not None:
                self.rmq.send(str({
                    'id': self.id,
                    'pos': list(self.pos),
                    'angle': self.angle
                }))
            sleep(Agv.LOOP_TIME)

    def stop(self):
        self.running = False
        self.thread.join()

# for debugging
if __name__ == "__main__":
    agv1 = Agv()
    sleep(1)
    agv1.target = np.array([30,0])
    sleep(2)
    agv1.stop()
