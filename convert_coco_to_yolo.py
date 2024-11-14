from ultralytics import YOLO
from ultralytics.data.converter import convert_coco
convert_coco(labels_dir='coco_dataset/annotation/', use_segments=True)