import cv2
import tensorflow as tf

def load_model(model_path, graph):
    with graph.as_default():
        graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(model_path, 'rb') as gf:
            serial_g = gf.read()
            graph_def.ParseFromString(serial_g)
            tf.import_graph_def(graph_def, name='')

def load_graph(graph_file):
    """Loads a frozen inference graph"""
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return graph

def process_image(image):
    # resized = cv2.resize(image, (300, 300))
    # img     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return img

