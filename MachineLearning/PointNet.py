import os
import glob
import trimesh
import numpy as np
import tensorflow as tf
from tensorflow import keras
from keras import layers
from matplotlib import pyplot as plt

tf.random.set_seed(1234)

DATA_DIR = ""
CLASS_MAP = None

# Set the number of points to sample and batch size 
NUM_POINTS = 2048
NUM_CLASSES = 10
BATCH_SIZE = 32
train_dataset = None
test_dataset = None

def visualize_chair():
    # We can use the trimesh package to read and visualize the .off mesh files.
    mesh = trimesh.load(os.path.join(DATA_DIR, "chair/train/chair_0001.off"))
    mesh.show()

    # To convert a mesh file to a point cloud we first need to sample points on the mesh surface.
    # .sample() performs a unifrom random sampling. Here we sample at 2048 locations and visualize in matplotlib.

    points = mesh.sample(2048)

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])
    ax.set_axis_off()
    plt.show()

def load_dataset():
    global DATA_DIR
    # Load dataset
    # We use the ModelNet10 model dataset, the smaller 10 class 
    # version of the ModelNet40 dataset. First download the data (only if doesn't currently exist):
    DATA_DIR = tf.keras.utils.get_file(
        "modelnet.zip",
        "http://3dvision.princeton.edu/projects/2014/3DShapeNets/ModelNet10.zip",
        extract=True,
    )

    DATA_DIR = os.path.join(os.path.dirname(DATA_DIR), "ModelNet10")

# Data augmentation is important when working with point cloud data. We create a augmentation
# function to jitter and shuffle points within a class.
def jitter_and_shuffle_points(points, label):
    # jitter points
    points += tf.random.uniform(points.shape, -0.005, 0.005, dtype=tf.float64)
    # shuffle points
    points = tf.random.shuffle(points)
    return points, label

# To generate a tf.data.Dataset() we need to first parse through the ModelNet data folders.
# Each mesh is loaded and sampled into a point cloud before being added to a standard python
# list and converted to a numpy array. We also store the current enumerate index value as the
# object label and use a dictionary to recall this later.
def parse_dataset(num_points=2048):
    global CLASS_MAP
    global train_dataset
    global test_dataset

    print("in parse_dataset()")
    train_points = []
    train_labels = []
    test_points = []
    test_labels = []
    CLASS_MAP = {}
    folders = glob.glob(os.path.join(DATA_DIR, "[!README]*"))

    for i, folder in enumerate(folders):
        print("processing class: {}".format(os.path.basename(folder)))
        # store folder name with ID so we can retrieve later
        CLASS_MAP[i] = folder.split("/")[-1]
        # gather all files
        train_files = glob.glob(os.path.join(folder, "train/*"))
        test_files = glob.glob(os.path.join(folder, "test/*"))

        for f in train_files:
            train_points.append(trimesh.load(f).sample(num_points))
            train_labels.append(i)

        for f in test_files:
            test_points.append(trimesh.load(f).sample(num_points))
            test_labels.append(i)

    train_points = np.array(train_points)
    test_points = np.array(test_points)
    train_labels = np.array(train_labels)
    test_labels = np.array(test_labels)

    # Our data can now be read into a tf.data.Dataset() object. 
    train_dataset = tf.data.Dataset.from_tensor_slices((train_points, train_labels))
    test_dataset = tf.data.Dataset.from_tensor_slices((test_points, test_labels))

    # We set the shuffle buffer size to the entire size of the dataset as prior to this the data is ordered by class.
    # The first shuffle is not shuffling points, it's shuffling the order of the inputs (sets of points)
    train_dataset = train_dataset.shuffle(len(train_points)).map(jitter_and_shuffle_points).batch(BATCH_SIZE)
    test_dataset = test_dataset.shuffle(len(test_points)).batch(BATCH_SIZE)

def build_model():
    # Build a model

    # Each convolution and fully-connected layer (with exception for end layers) 
    # consits of Convolution / Dense -> Batch Normalization -> ReLU Activation.

    def conv_bn(x, filters):
        x = layers.Conv1D(filters, kernel_size=1, padding="valid")(x)
        x = layers.BatchNormalization(momentum=0.0)(x)
        return layers.Activation("relu")(x)

    def dense_bn(x, filters):
        x = layers.Dense(filters)(x)
        x = layers.BatchNormalization(momentum=0.0)(x)
        return layers.Activation("relu")(x)

    # PointNet consists of two core components. The primary MLP network, and the transformer net (T-net). 
    # The T-net aims to learn an affine transformation matrix by its own mini network. The T-net is used twice. 
    # The first time to transform the input features (n, 3) into a canonical representation. 
    # The second is an affine transformation for alignment in feature space (n, 3). 
    # As per the original paper we constrain the transformation to be close to an
    # orthogonal matrix (i.e. ||X*X^T - I|| = 0).

    # penalizes being not orthogonal. This is because we have a transformation by a 64x64 matrix.
    # if the matrix isn't orthogonal, then the columns aren't a 
    class OrthogonalRegularizer(keras.regularizers.Regularizer):
        def __init__(self, num_features, l2reg=0.001):
            self.num_features = num_features
            self.l2reg = l2reg
            self.eye = tf.eye(num_features)

        def __call__(self, x):
            x = tf.reshape(x, (-1, self.num_features, self.num_features))
            xxt = tf.tensordot(x, x, axes=(2, 2))
            xxt = tf.reshape(xxt, (-1, self.num_features, self.num_features))
            return tf.reduce_sum(self.l2reg * tf.square(xxt - self.eye))

        def get_config(self):
            config = {
                'num_features': self.num_features,
                'l2reg': self.l2reg,
            }
            return config

    # We can then define a general function to build T-net layers.

    def tnet(inputs, num_features):
        # Initalise bias as the indentity matrix
        bias = keras.initializers.Constant(np.eye(num_features).flatten())
        reg = OrthogonalRegularizer(num_features)

        x = conv_bn(inputs, 32)
        x = conv_bn(x, 64)
        x = conv_bn(x, 512)
        x = layers.GlobalMaxPooling1D()(x)
        x = dense_bn(x, 256)
        x = dense_bn(x, 128)
        x = layers.Dense(
            num_features * num_features,
            kernel_initializer="zeros",
            bias_initializer=bias,
            activity_regularizer=reg,
        )(x)
        feat_T = layers.Reshape((num_features, num_features))(x)
        # Apply affine transformation to input features
        return layers.Dot(axes=(2, 1))([inputs, feat_T])

    # The main network can be then implemented in the same manner where the t-net mini models
    # can be dropped in a layers in the graph. Here we replicate the network architecture published
    # in the original paper but with half the number of weights at each layer as we are using the 
    # smaller 10 class ModelNet dataset.
    inputs = keras.Input(shape=(NUM_POINTS, 3)) # (0, 2048, 3)

    x = tnet(inputs, 3) # learn a 3by3 matrix to remove rotation + scale variation (0, 2048, 3)

    # mlp (basically fully connected layer applied to each point)
    x = conv_bn(x, 32) # (0, 2048, 32)
    x = conv_bn(x, 32) # (0, 2048, 32)

    # learn a 32 by 32 matrix to transform each point to have 32 dimensions which each 
    # dimension could be like which class the point is in or something I think
    x = tnet(x, 32) # (0, 2048, 32)

    # mlp (basically fully connected layer applied to each point)
    x = conv_bn(x, 32) # (0, 2048, 32)
    x = conv_bn(x, 64) # (0, 2048, 64)
    x = conv_bn(x, 512) # (0, 2048, 512)

    # idk what the max pool is for. Is it to get the most likely class each point is in?
    x = layers.GlobalMaxPooling1D()(x) # (0, 2048)

    # fully connected at the end to get the output class
    x = dense_bn(x, 256) # (0, 256)
    x = layers.Dropout(0.3)(x)
    x = dense_bn(x, 128) # (0, 128)
    x = layers.Dropout(0.3)(x)

    outputs = layers.Dense(NUM_CLASSES, activation="softmax")(x) # (0, 10)

    model = keras.Model(inputs=inputs, outputs=outputs, name="pointnet")
    model.summary()

    return model

# first load the dataset
load_dataset()

# parse the dataset. This can take ~5 minutes to complete.
parse_dataset(NUM_POINTS)

model = build_model()
from keras_visualizer import visualizer
visualizer(model, file_format='png', view=True)


# Train model
# Once the model is defined it can be trained like any other standard 
# classification model using .compile() and .fit().
model.compile(
    loss="sparse_categorical_crossentropy",
    optimizer=keras.optimizers.Adam(learning_rate=0.001),
    metrics=["sparse_categorical_accuracy"],
)

model.fit(train_dataset, epochs=20, validation_data=test_dataset)

data = test_dataset.take(1)

points, labels = list(data)[0]
points = points[:8, ...]
labels = labels[:8, ...]

# run test data through model
preds = model.predict(points)
preds = tf.math.argmax(preds, -1)

points = points.numpy()

# plot points with predicted class and label
fig = plt.figure(figsize=(15, 10))
for i in range(8):
    ax = fig.add_subplot(2, 4, i + 1, projection="3d")
    ax.scatter(points[i, :, 0], points[i, :, 1], points[i, :, 2])
    ax.set_title(
        "pred: {:}, label: {:}".format(
            CLASS_MAP[preds[i].numpy()], CLASS_MAP[labels.numpy()[i]]
        )
    )
    ax.set_axis_off()
plt.show()
