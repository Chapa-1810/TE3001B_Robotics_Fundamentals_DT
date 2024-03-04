#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
import spacy

figures = {
  "esfera": 1,
  "cubo": 2,
  "cuboide": 3,
  "cilindro": 4,
  "hexagonal": 5,
  "triangular": 6,
  "cono": 7,
  "piramide": 8,
  "tetaedro": 9
}

class NLP(Node):
  def __init__(self):
    super().__init__('nlp')
    self.create_subscription(String, 'hri/whisper_text', self.text_callback, 10)
    self.publisher = self.create_publisher(Int16, 'hri/figure', 10)

    self.nlp = spacy.load('es_dep_news_trf')

    self.msg = Int16()

    self.get_logger().info('NLP node has been started')

  def text_callback(self, msg):
    global figures
    doc = self.nlp(msg.data)

    verb_pos = 0
    target = 0

    for pos, token in enumerate(doc):
      if token.pos_ == "VERB":
          verb_pos = pos

    predicate = doc[verb_pos:]

    for pos, token in enumerate(predicate):
      if ((token.pos_ == "NOUN") or (token.pos_ == "PROPN")) and (token.text in figures) and (token.text.lower() != "prisma"):
        target = figures[token.text]
        break

    self.get_logger().info('Target shape: "%s"' % target)

    self.msg.data = target
    self.publisher.publish(self.msg)

def main(args=None):
  rclpy.init(args=args)

  nlp_node = NLP()

  rclpy.spin(nlp_node)

  nlp_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

