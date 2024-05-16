#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time
import sys
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main_process(image):
    global INPUT_WIDTH, INPUT_HEIGHT, SCORE_THRESHOLD, NMS_THRESHOLD,CONFIDENCE_THRESHOLD 
    INPUT_WIDTH = 640
    INPUT_HEIGHT = 640
    SCORE_THRESHOLD = 0.2
    NMS_THRESHOLD = 0.4
    CONFIDENCE_THRESHOLD = 0.4
    class_list = load_classes()
    colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0),(244,220,110)]
    is_cuda = True
    #len(sys.argv) > 2 and sys.argv[2] == "cuda"
    net = build_model(is_cuda)
    #capture = load_capture()
    capture = image

    frame_count = 0
    total_frames = 0
    fps = -1

    while True:

        _, frame = capture.read()
        if frame is None:
            print("End of stream")
            break

        inputImage = format_yolov5(frame)
        outs = detect(inputImage, net)

        class_ids, confidences, boxes = wrap_detection(inputImage, outs[0])

        frame_count += 1
        total_frames += 1

        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            color = colors[int(classid) % len(colors)]
            cv2.rectangle(frame, box, color, 2)
            cv2.rectangle(frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
            cv2.putText(frame, class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
            print(class_list[classid])
            pub.publish(class_list[classid])

        if frame_count >= 30:
            end = time.time_ns()
            fps = 1
            frame_count = 0
        
        if fps > 0:
            fps_label = "FPS: %.2f" % fps
            cv2.putText(frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("output", frame)

        if cv2.waitKey(1) > -1:
            print("finished by user")
            break

    print("Total frames: " + str(total_frames))

    capture.release()
    cv2.destroyAllWindows()
    cv2.imshow("Red", image)
    cv2.waitKey(1)

def imageCallback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        #cv2.imshow("Image Subscriber PY",cv_image)
        #cv2.waitKey(1)
        main_process(cv_image)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s",e)

def build_model(is_cuda):
    net = cv2.dnn.readNetFromONNX("best.onnx")
    if is_cuda:
        print("Attempty to use CUDA")
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
    else:
        print("Running on CPU")
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    return net

def detect(image, net):
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
    net.setInput(blob)
    preds = net.forward()
    return preds

def load_capture():
    # Si se proporciona un archivo de video como argumento en línea de comandos, lo carga
    video_path = 0
    #puerto de la camara = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'
    if len(sys.argv) > 1:
        video_path = sys.argv[1]
        capture = cv2.VideoCapture(video_path)
        print("Loaded video from:", video_path)
    # Si no se proporciona un archivo de video, captura desde la cámara en tiempo real
    else:
        capture = cv2.VideoCapture(video_path)
        print("Started capturing from camera")
    return capture

def load_classes():
    class_list = []
    with open("/home/ximena/Documentos/octavo/catkin_ws/src/activity_1/scripts/classes.txt", "r") as f:
        class_list = [cname.strip() for cname in f.readlines()]
    return class_list

def wrap_detection(input_image, output_data):
    class_ids = []
    confidences = []
    boxes = []

    rows = output_data.shape[0]

    image_width, image_height, _ = input_image.shape

    x_factor = image_width / INPUT_WIDTH
    y_factor =  image_height / INPUT_HEIGHT

    for r in range(rows):
        row = output_data[r]
        confidence = row[4]
        if confidence >= 0.4:

            classes_scores = row[5:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > .25):

                confidences.append(confidence)

                class_ids.append(class_id)

                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                box = np.array([left, top, width, height])
                boxes.append(box)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

    result_class_ids = []
    result_confidences = []
    result_boxes = []

    for i in indexes:
        result_confidences.append(confidences[i])
        result_class_ids.append(class_ids[i])
        result_boxes.append(boxes[i])

    return result_class_ids, result_confidences, result_boxes

def format_yolov5(frame):
    row, col, _ = frame.shape
    _max = max(col, row)
    result = np.zeros((_max, _max, 3), np.uint8)
    result[0:row, 0:col] = frame
    return result


if __name__ == "__main__":
    rospy.init_node('red')
    pub = rospy.Publisher('/survey',String, queue_size=10)
    nodeRate = 100
    rate = rospy.Rate(nodeRate)
    rospy.Subscriber("/camera/image_raw", Image, imageCallback)
    rospy.spin()