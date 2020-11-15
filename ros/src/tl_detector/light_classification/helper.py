import cv2
import tensorflow as tf

def load_model(model_path, graph):
    with graph.as_default():
        graph_def = tf.GraphDef()
        with tf.gfile.GFile(model_path, 'rb') as gf:
            serial_g = gf.read()
            graph_def.ParseFromString(serial_g)
            tf.import_graph_def(graph_def, name='')

def process_image(image):
    img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return img

