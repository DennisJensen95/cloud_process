import torchvision
import torchvision.transforms as T
import cv2
from PIL import Image
import numpy as np
import torch
from math import floor

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
print(f'Neural net running on: {device}')
model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True).to(device)
# torchvision.models.

# model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=True).to(device)
model.eval()
COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

# normalize = T.Normalize(mean=[0.485, 0.456, 0.406],
#                                  std=[0.229, 0.224, 0.225])
down_sample = 1
def get_prediction(img):
  img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  img = Image.fromarray(img) # Load the image
  img_temp = img
  transform = T.Compose([T.ToTensor()]) # Defing PyTorch Transform
  img = transform(img).to(device) # Apply the transform to the image
  pred = model([img]) # Pass the image to the model
  boxes = [[(i[0], i[1]), (i[2], i[3])] for i in list(pred[0]['boxes'].detach().cpu().numpy())]
  scores = pred[0]['scores'].detach().cpu().numpy()
  labels = pred[0]['labels'].cpu().numpy()
  return boxes, scores, labels, img_temp

def fast_pred(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(img)
    transform = T.Compose([T.ToTensor()])
    img = transform(img).to(device)
    pred = model([img])
    boxes = [[(i[0], i[1]), (i[2], i[3])] for i in list(pred[0]['boxes'].detach().cpu().numpy())]
    scores = pred[0]['scores'].detach().cpu().numpy()
    labels = pred[0]['labels'].cpu().numpy()
    return boxes, scores, labels

def object_detection_api(img, objects_to_detect, threshold=0.5, rect_th=3, text_size=3, text_th=3, label=False):
    boxes, pred_cls, labels, img_downsampled = get_prediction(img) # Get predictions
    if label:
        if len(pred_cls) > 0:
            idx = np.argmax(pred_cls)
            return COCO_INSTANCE_CATEGORY_NAMES[labels[idx]], pred_cls[idx]
        else:
            return None
    for i in range(len(boxes)):
        for j in range(len(objects_to_detect)):
            if ('sports ball' == COCO_INSTANCE_CATEGORY_NAMES[labels[i]] or
                'orange' == COCO_INSTANCE_CATEGORY_NAMES[labels[i]]) and pred_cls[i] > threshold:
                print(f'{COCO_INSTANCE_CATEGORY_NAMES[labels[i]]} - score: {pred_cls[i]}')
                left_upper_corner = (int(floor(boxes[i][0][0])), int(floor(boxes[i][0][1])))
                right_lower_corner = (int(floor(boxes[i][1][0])), int(floor(boxes[i][1][1])))
                left_lower_corner = (int(floor(boxes[i][0][0])), int(floor(boxes[i][1][1])))
                right_upper_corner = (int(floor(boxes[i][1][0])), int(floor(boxes[i][0][1])))
                corners = [left_upper_corner, left_lower_corner, right_lower_corner, right_upper_corner]
                side_length_x = abs(int(floor(corners[0][0] - corners[3][0])))
                side_length_y = abs(int(floor(corners[0][1] - corners[1][1])))
                side_length_avg = int((side_length_x + side_length_y) / 2)
                corners[3] = (corners[0][0] + side_length_avg, corners[0][1])
                corners[2] = (corners[0][0] + side_length_avg, corners[0][1] + side_length_avg)
                corners[1] = (corners[0][0], corners[0][1] + side_length_avg)
                cv2.rectangle(img, corners[0], corners[2], color=(0, 255, 0), thickness=rect_th) # Draw Rectangle with the coordinates
                # cv2.putText(img, pred_cls[i], boxes[i][0],  cv2.FONT_HERSHEY_SIMPLEX, text_size, (0, 255, 0), thickness=1) # Write the prediction class
                # center = tuple((np.asarray(int(boxes[i][0]*1/down_sample)) + np.asarray(int(boxes[i][1]*1/down_sample))) / 2)
                # cv2.circle(img, center, radius=5, color=(255, 0, 0), thickness=5)

    return img

def give_box_corners_of_ball(img):
    boxes, pred_cls, labels = fast_pred(img)
    balls = []
    for i in range(len(boxes)):
        if ('sports ball' == COCO_INSTANCE_CATEGORY_NAMES[labels[i]] or
            'orange' == COCO_INSTANCE_CATEGORY_NAMES[labels[i]]) and pred_cls[i] > 0.001:
            left_upper_corner = (int(floor(boxes[i][0][0])), int(floor(boxes[i][0][1])))
            right_lower_corner = (int(floor(boxes[i][1][0])), int(floor(boxes[i][1][1])))
            left_lower_corner = (int(floor(boxes[i][0][0])), int(floor(boxes[i][1][1])))
            right_upper_corner = (int(floor(boxes[i][1][0])), int(floor(boxes[i][0][1])))
            corners = [left_upper_corner, left_lower_corner, right_lower_corner, right_upper_corner]
            side_length_x = abs(int(floor(corners[0][0] - corners[3][0])))
            side_length_y = abs(int(floor(corners[0][1] - corners[1][1])))
            side_length_avg = int((side_length_x + side_length_y) / 2)
            corners[3] = (corners[0][0] + side_length_avg, corners[0][1])
            corners[2] = (corners[0][0] + side_length_avg, corners[0][1] + side_length_avg)
            corners[1] = (corners[0][0], corners[0][1] + side_length_avg)
            balls.append([corners[0], corners[1], corners[2], corners[3]])

    return balls