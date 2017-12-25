import tensorflow as tf
import numpy as np
import cv2
import os

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

class UavDect():
    def __init__(self):

        self.PATH_TO_CKPT = 'frozen_inference_graph.pb'
        self.PATH_TO_LABELS = 'label_map.pbtxt'
        self.NUM_CLASSES = 1

        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=self.NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

        self.DETECTION_GRAPH = None
        self.SESS = None
        self.setup()

    def setup(self):
        global SESS, DETECTION_GRAPH
        self.DETECTION_GRAPH = tf.Graph()
        with DETECTION_GRAPH.as_default():
            self.od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                self.od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(self.od_graph_def, name='')

            self.config = tf.ConfigProto()
            self.config.gpu_options.allow_growth = True
            SESS = tf.Session(graph=DETECTION_GRAPH, config=self.config)

    def detect(self,image):
        image_np = image
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = DETECTION_GRAPH.get_tensor_by_name('image_tensor:0')
        boxes = DETECTION_GRAPH.get_tensor_by_name('detection_boxes:0')
        scores = DETECTION_GRAPH.get_tensor_by_name('detection_scores:0')
        classes = DETECTION_GRAPH.get_tensor_by_name('detection_classes:0')
        num_detections = DETECTION_GRAPH.get_tensor_by_name('num_detections:0')
        (boxes, scores, classes, num_detections) = SESS.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            self.category_index,
            use_normalized_coordinates=True,
            min_score_thresh =0.6,
            line_thickness=8)
        return (image_np, boxes[0], scores[0],classes[0])

# def test():
#     i=0
#     filelist = os.listdir(os.getcwd()+'/test_jpgs')
#     for filename in filelist:
#         if filename.endswith('.jpg'):
#             image = cv2.imread(filename)
#             cv2.imwrite(str(i)+'.jpg',detect(image)[0])
#             i = i+1
#
# if __name__ == "__main__":
#     setup()
#     test()
    #cap = cv2.VideoCapture(1)
    # while(1):
    #     frame=cv2.imread('000072.jpg')
    #     detFrame = detect(frame)[0]
    #     cv2.imshow('ff',detFrame)
    #     cv2.waitKey(1)

