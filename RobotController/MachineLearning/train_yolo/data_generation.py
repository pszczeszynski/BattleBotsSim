import numpy as np
import shutil
import json
import os
import cv2
import sys
from importlib import reload
from multiprocessing import Pool

sys.path.append('..')
import Utilities
reload(Utilities)
from Utilities import prepare_and_augment_image, Augmentations
sys.path.remove('..')

train_size = 35000
test_size = 15000
data_dir = "set_2.0_new_model"

def transform_matrix_offset_center(matrix, x, y):
    o_x = float(x) / 2 - 0.5
    o_y = float(y) / 2 - 0.5
    offset_matrix = np.array([[1, 0, o_x], [0, 1, o_y], [0, 0, 1]])
    reset_matrix = np.array([[1, 0, -o_x], [0, 1, -o_y], [0, 0, 1]])
    transform_matrix = np.dot(np.dot(offset_matrix, matrix), reset_matrix)
    return transform_matrix

def apply_affine_transform(theta=0, tx=0, ty=0, shear=0, zx=1, zy=1, row_axis=2, col_axis=1):
    theta = np.deg2rad(theta)
    rotation_matrix = np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]
    )
    transform_matrix = rotation_matrix

    shift_matrix = np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]])
    transform_matrix = np.dot(transform_matrix, shift_matrix)

    shear = np.deg2rad(shear)
    shear_matrix = np.array(
        [[1, -np.sin(shear), 0], [0, np.cos(shear), 0], [0, 0, 1]]
    )
    transform_matrix = np.dot(transform_matrix, shear_matrix)

    zoom_matrix = np.array([[zx, 0, 0], [0, zy, 0], [0, 0, 1]])
    transform_matrix = np.dot(transform_matrix, zoom_matrix)

    transform_matrix = transform_matrix_offset_center(
        transform_matrix, 720, 720
    )

    if col_axis > row_axis:
        transform_matrix[:, [0, 1]] = transform_matrix[:, [1, 0]]
        transform_matrix[[0, 1]] = transform_matrix[[1, 0]]

    return transform_matrix

augs = Augmentations(
    zoom_range = 0.3,
    width_shift_range = 0.2,
    height_shift_range = 0.2,
    rotation_range = 0,
    brightness_range = [0.7, 1.3],
    max_overlay_objects = 100,
    object_size = [24, 24],
    blur_probability = 0.2,
)

def move_data(img_dir, i):
    img_name = f'image_{i}.jpg'
    key_name = f'image_{i}.json'
    
    shutil.copy(f'../TrainingData/TrainingKeys/{key_name}', f'datasets/{data_dir}/keys/')
    with open(f"datasets/{data_dir}/keys/{key_name}", "r") as json_file:
        json_data = json.load(json_file)

    x = int(json_data["transformedPoint"]["x"])
    y = int(json_data['transformedPoint']['y'])
    w = int(json_data["transformedPoint"]["z"])
    h = int(json_data["transformedPoint"]["w"])

    img = cv2.imread(f'../TrainingData/TrainingInputsPosition/{img_name}')
    aug_img, params = prepare_and_augment_image(img, augs)
    aug_img = (aug_img*255).astype(np.uint8) 
    img_gray = cv2.cvtColor(aug_img, cv2.COLOR_BGR2GRAY)

    theta = params['theta']
    tx = params['tx']
    ty = params['ty']
    zx = params['zx']
    zy = params['zy']
    shear = params['shear']

    matrix = apply_affine_transform(theta, tx, ty, shear, zx, zy, row_axis=1, col_axis=2)
    a = matrix[0,0]
    b = matrix[0,1]
    c = matrix[0,2]
    d = matrix[1,0]
    e = matrix[1,1]
    f = matrix[1,2]

    x1 = (x-f-d*y)/e
    y1 = (y-c-b*x)/a
    w /= zx
    h /= zy

    bad = False
    if x1 - w/2 < 0:
        if x1 + w/2 > 0:
            x1 = 0 + (x1 + w/2 - 0) / 2
            w = x1 + w/2 - 0
        else:
            bad = True
    if x1 + w/2 > 720:
        if x1 - w/2 < 720:
            x1 = 720 - (720 - x1 + w/2) / 2
            w = 720 - x1 + w/2
        else:
            bad = True
    if y1 - h/2 < 0:
        if y1 + h/2 > 0:
            y1 = (y1 + h/2) / 2
            h = y1 + h/2
        else:
            bad = True
    if y1 + h/2 > 720:
        if y1 - h/2 < 720:
            y1 = 720 - (720 - y1 + h/2) / 2
            h = 720 - y1 + h/2
        else:
            bad = True
    if bad:
        return False

    cv2.imwrite(f'datasets/{data_dir}/{img_dir}/images/{img_name}', img_gray)

    with open(f"datasets/{data_dir}/{img_dir}/labels/image_{i}.txt", "w") as file:
        file.write(f'0 {x1/720} {y1/720} {w/720} {h/720}')

    return True

def move_data_no_aug(img_dir, i):
    img_name = f'image_{i}.jpg'
    key_name = f'image_{i}.json'
    
    with open(f"datasets/{data_dir}/keys/{key_name}", "r") as json_file:
        json_data = json.load(json_file)

    x = int(json_data["transformedPoint"]["x"])
    y = int(json_data['transformedPoint']['y'])
    w = int(json_data["transformedPoint"]["z"])
    h = int(json_data["transformedPoint"]["w"])

    img = cv2.imread(f'../TrainingData/TrainingInputsPosition/{img_name}')
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    bad = False
    if x - w/2 < 0:
        if x + w/2 > 0:
            x = 0 + (x + w/2 - 0) / 2
            w = x + w/2 - 0
        else:
            bad = True
    if x + w/2 > 720:
        if x - w/2 < 720:
            x = 720 - (720 - x + w/2) / 2
            w = 720 - x + w/2
        else:
            bad = True
    if y - h/2 < 0:
        if y + h/2 > 0:
            y = (y + h/2) / 2
            h = y + h/2
        else:
            bad = True
    if y + h/2 > 720:
        if y - h/2 < 720:
            y = 720 - (720 - y + h/2) / 2
            h = 720 - y + h/2
        else:
            bad = True
    if bad:
        return False

    cv2.imwrite(f'datasets/{data_dir}/{img_dir}/images/{img_name}', img_gray)

    with open(f"datasets/{data_dir}/{img_dir}/labels/image_{i}.txt", "w") as file:
        file.write(f'0 {x/720} {y/720} {w/720} {h/720}')

    return True

def move_no_transform(img_dir, i):
    img_name = f'image_{i}.jpg'
    
    img = cv2.imread(f'../TrainingData/TrainingInputsPosition/{img_name}')
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cv2.imwrite(f'datasets/{data_dir}/{img_dir}/images/{img_name}', img_gray)

    with open(f"datasets/{data_dir}/{img_dir}/labels/image_{i}.txt", "w") as file:
        file.close()

    return True


def train_test_split(train_size, test_size):
    os.makedirs(f'datasets/{data_dir}/train/images', exist_ok=True)
    os.makedirs(f'datasets/{data_dir}/train/labels', exist_ok=True)
    os.makedirs(f'datasets/{data_dir}/val/images', exist_ok=True)
    os.makedirs(f'datasets/{data_dir}/val/labels', exist_ok=True)
    os.makedirs(f'datasets/{data_dir}/keys', exist_ok=True)

    count = 0
    for i in range(train_size):
        if not move_data('train', i):
            count += 1
            if not move_data_no_aug('train', i):
                move_no_transform('train', i)
        if i%10000 == 0:
            print(i)
    print(count)

    count = 0
    for i in range(train_size, train_size+test_size):
        if not move_data('val', i):
            count += 1
            if not move_data_no_aug('val', i):
                move_no_transform('val', i)
        if i%2500 == 0:
            print(i)
    print(count)

# input size of training set and validation set, testing set not included yet
train_test_split(train_size, test_size)
print('Done')
