import pika, sys, os
from threading import Thread

class RabbitMqReceiver:
    def __init__(self, queue="hello", myCallback=None):
        self.channel = None
        self.queue = queue
        self.onMessage = myCallback
        self.thread = Thread(target=self.start)
        self.thread.start()

    def start(self):
        credentials = pika.PlainCredentials('rabbitmq', 'rabbitmq')
        connection = pika.BlockingConnection(pika.ConnectionParameters('localhost', credentials=credentials))
        self.channel = connection.channel()

        self.channel.queue_declare(queue=self.queue)

        def callback(ch, method, properties, body):
            print(" [x] Received %r" % body)
            if self.onMessage is not None:
                self.onMessage(body)

        self.channel.basic_consume(queue=self.queue, on_message_callback=callback, auto_ack=True)
        self.channel.start_consuming()

    def stop(self):
        self.agv.stop()
        self.channel.close()
        self.thread.join()

    