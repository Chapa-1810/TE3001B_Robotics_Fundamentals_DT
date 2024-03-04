#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
from figure_msgs.msg import Figure
import spacy
from spacy_utils import number_dictionary, figures

"""
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
"""

class NLP(Node):
  def __init__(self):
    super().__init__('nlp')
    self.create_subscription(String, 'hri/whisper_text', self.text_callback, 10)
    self.publisher = self.create_publisher(Figure, 'hri/figure', 10)

    self.nlp = spacy.load('es_dep_news_trf')

    self.msg = Figure()
    self.msg.header.frame_id = 'nlp'

    self.get_logger().info('NLP node has been started')

  def text_callback(self, msg):
    global figures
    doc = self.nlp(msg.data)

    verb_pos = 0
    specs_pos = 0
    target = 0
    length = 0
    radius = 0
    width = 0

    for pos, token in enumerate(doc):
      if token.pos_ == "VERB":
          verb_pos = pos

    predicate = doc[verb_pos+1:]
    self.get_logger().info('Predicate: "%s"' % predicate)

    for pos, token in enumerate(predicate):
      if ((token.pos_ == "NOUN") or (token.pos_ == "PROPN")):
        if token.text.lower().strip() == "prisma":
          self.get_logger().info('Hit prisma')
          for subpos, subtoken in enumerate(predicate[pos:]):
            if subtoken.pos_ == "ADJ":
              target = figures[subtoken.text]
              self.get_logger().info('Target prisma: "%s"' % subtoken.text)
              specs_pos = pos+1
              break
        else:
          target = figures[token.text]
          self.get_logger().info('Target text: "%s"' % token.text)
          specs_pos = pos
        break

    specs = predicate[specs_pos+1:]
    self.get_logger().info('Specs: "%s"' % specs)

    for pos, token in enumerate(specs):
      # Num or like num
      if token.pos_ == "ADJ":
        #self.get_logger().info('Hit ADJ "%s"' % token.text)
        if token.text.lower().strip() == "largo":
          for subpos, subtoken in enumerate(specs[pos:]):
            self.get_logger().info('Subtoken: "%s"' % subtoken.text)
            if subtoken.pos_ == "NUM" or subtoken.like_num:
                if subtoken.is_digit:
                  length = float(subtoken.text)
                else:
                  length = float(number_dictionary[subtoken.text])
                break
        elif token.text.lower().strip() == "ancho":
          for subpos, subtoken in enumerate(specs[pos:]):
            self.get_logger().info('Subtoken: "%s"' % subtoken.text)
            if subtoken.pos_ == "NUM" or subtoken.like_num:
                if subtoken.is_digit:
                  width = float(subtoken.text)
                else:
                  width = float(number_dictionary[subtoken.text])
                break
      elif token.pos_ == "NOUN" and token.text.lower().strip() == "radio":
        #self.get_logger().info('Hit Radius "%s"' % token.text)
        for subpos, subtoken in enumerate(specs[pos:]):
          self.get_logger().info('Subtoken: "%s"' % subtoken.text)
          if subtoken.pos_ == "NUM" or subtoken.like_num:
            if subtoken.is_digit:
              radius = float(subtoken.text)
            else:
              radius = float(number_dictionary[subtoken.text])
            break
    for pos, token in enumerate(predicate):
      self.get_logger().info('Token: "%s", POS: "%s"' % (token.text, token.pos_))

    self.get_logger().info('Target shape: "%s"' % target)

    self.get_logger().info('Target specs:')
    self.get_logger().info('Radius: "%s"' % radius)
    self.get_logger().info('Width: "%s"' % width)
    self.get_logger().info('Length: "%s"' % length)

    self.msg.figure_id = target
    self.msg.header.stamp = self.get_clock().now().to_msg()

    self.msg.radius = float(radius)
    self.msg.width  = float(width)
    self.msg.length = float(length)
    
    self.publisher.publish(self.msg)

def main(args=None):
  rclpy.init(args=args)

  nlp_node = NLP()

  rclpy.spin(nlp_node)

  nlp_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

