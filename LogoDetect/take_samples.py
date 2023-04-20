import cv2
import torch
import numpy as np

# --------------------------------
# 1. Load yolov5 model and some functions for running the model
obj_model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5_model/obj.pt', force_reload=True)
obj_model.iou = 0.5
obj_model.conf = 0.8
obj_classes = obj_model.names


def yolo_detect(I, model):
    result = model([I], size=320)
    # cordinates is: x1, y2, x2, y2, confidence
    labels, cordinates = result.xyxyn[0][:, -1], result.xyxyn[0][:, :-1]
    return labels, cordinates


def yolo_bb_obj(result, frame, classes):
    labels, cord = result
    x_shape, y_shape = frame.shape[1], frame.shape[0]
    cropped_logo = None
    bbox = (0, 0, 0, 0)
    for i in range(len(labels)):
        row = cord[i]
        x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(row[3] * y_shape)
        cls = classes[int(labels[i])]
        if cls == 'logo':
            cropped_logo = frame[y1 + 3:y2 - 3, x1 + 3:x2 - 3]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f" {round(float(row[4]), 2)}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)
        elif cls == 'dep':
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, f" {round(float(row[4]), 2)}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)
            bbox = (x1, y1, x2, y2)
    return frame, cropped_logo, bbox


def detect_object(input_queue):
    try:
        img_org = input_queue.copy()
        results = yolo_detect(cv2.cvtColor(input_queue, cv2.COLOR_BGR2RGB), model=obj_model)

        # detect both of logo and obj: {15:obj, 16:logo}
        if 15 in results[0] and 16 in results[0]:
            detected_frame, crop_logo, bb = yolo_bb_obj(results, img_org, classes=obj_classes)
            crop_logo = np.ascontiguousarray(crop_logo)
            return True, detected_frame, crop_logo, bb
        else:
            return False, img_org, None, (0, 0, 0, 0)
    except Exception as e:
        print(e, "-obj process")


# --------------------------------
# 2. Capture new data.

vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)

count = 0
while True:
    _, img = vid.read()
    isObject, yolo_img, logo, _ = detect_object(img)

    cv2.imshow("img", yolo_img)
    if isObject:
        logo = cv2.resize(logo, (124, 124))
        cv2.imshow("logo", logo)

    key = cv2.waitKey(1)
    if key == ord('q'):
        if isObject:
            cv2.imwrite(f"datasets/not_error/lo07_{count}.jpg", logo)
            print(f"Image: {count}")
            count += 1
    if key == ord('e'):
        vid.release()
        cv2.destroyAllWindows()
        break
