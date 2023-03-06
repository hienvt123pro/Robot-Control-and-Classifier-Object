import cv2
import torch
from find_feature import find_features
from preprocessing import preprocessing_img

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
    for i in range(len(labels)):
        row = cord[i]
        x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(row[3] * y_shape)
        cls = classes[int(labels[i])]
        if cls == 'dep':
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, f" {round(float(row[4]), 2)}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)
            return frame, tuple([x1, y1, x2, y2])


def detect_object(I):
    i = cv2.cvtColor(I, cv2.COLOR_BGR2RGB)
    results = yolo_detect(i, model=obj_model)
    # detect both of logo and obj: {15:obj, 16:logo}
    if not ((15 in results[0]) and (16 in results[0])):
        return I, tuple([0, 0, 0, 0])
    detect_frame, bounding_box = yolo_bb_obj(results, cv2.cvtColor(i, cv2.COLOR_RGB2BGR), classes=obj_classes)
    return detect_frame, bounding_box


# --------------------------------
# 2. Capture new data, save into Excel file on realtime.

""" create Excel file """
from openpyxl import Workbook

workbook = Workbook()
sheet1 = workbook.active
sheet1.title = "size"
sheet1['A1'] = 'd1'
sheet1['B1'] = 'd2'
sheet1['C1'] = 'd3'
sheet1['D1'] = 'label'


def create_workbook(path, index, d_1, d_2, d_3, labels):
    col_name = {1: 'A', 2: 'B', 3: 'C', 4: 'D'}
    col1name = col_name.get(1) + str(index)
    col2name = col_name.get(2) + str(index)
    col3name = col_name.get(3) + str(index)
    col4name = col_name.get(4) + str(index)
    sheet1[col1name] = d_1
    sheet1[col2name] = d_2
    sheet1[col3name] = d_3
    sheet1[col4name] = labels
    workbook.save(path)


vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)

count = 1
d1, d2, d3 = 0, 0, 0
label = [0, 1, 2, 3]  # sz 16, sz18, sz20, error
print("LABEL-->  0: sz16 | 1: sz18 | 2: sz20 | 3: error")
index_label = 0

while True:
    _, img = vid.read()
    _, (x1, y1, x2, y2) = detect_object(img)

    obj_img, pre_img = preprocessing_img(img, (x1, y1, x2, y2))
    obj_img, cen, d1, d2, d3 = find_features(obj_img, pre_img)

    cv2.imshow("org", obj_img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        """ save data to Excel """
        print('- Read a new frame {} with f1={}, f2={}, f3={}, center={}'.format(count, d1, d2, d3, cen))
        print('  label: {}'.format(index_label))
        create_workbook("new_datasets/train/size_data.xlsx", count + 1, d1, d2, d3, index_label)  # change path for train or test purpose
        count = count + 1
    if key == ord('c'):
        """ change label """
        index_label = 0 if index_label >= 3 else index_label + 1
        print('new label: {}'.format(index_label))
    if key == ord('e'):
        vid.release()
        cv2.destroyAllWindows()
        break
