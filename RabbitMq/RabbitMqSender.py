#!/usr/bin/env python
import pika

class RabbitMqSender:
    def __init__(self, queue="hello"):
        self.queue = queue
        credentials = pika.PlainCredentials('rabbitmq', 'rabbitmq')
        self.connection = pika.BlockingConnection(pika.ConnectionParameters('localhost', credentials=credentials))
        self.channel = self.connection.channel()
        self.channel.queue_declare(queue=queue)


    def send(self, msg):
        self.channel.basic_publish(exchange='',
                            routing_key=self.queue,
                            body=msg)

    def close(self):
        self.connection.close()