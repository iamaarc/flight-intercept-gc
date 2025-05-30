🚗 Object Detection in an Urban Environment
This project demonstrates how to build, train, and deploy an object detection model for urban driving scenarios using the TensorFlow Object Detection API and Amazon SageMaker.

At the end of this project, the model is capable of detecting vehicles, pedestrians, and cyclists in urban settings using Waymo Open Dataset sequences.

<p align="center"> <img src="data/animation.gif" alt="Urban Object Detection Demo" width="600"/> </p>
📦 Project Structure
This repository includes two Jupyter notebooks:

Notebook	Description
1_train_model.ipynb	Launches a training job on AWS SageMaker and streams TensorBoard logs
2_deploy_model.ipynb	Deploys the trained model, runs inference on validation/test data, and generates a GIF visualization

✅ Highlights
Trained an object detection model on the Waymo Open Dataset

Used EfficientDet D1 as the primary pretrained model architecture

Fine-tuned training hyperparameters (batch size, steps, learning rate) using the pipeline.config file

Evaluated performance using Mean Average Precision (mAP) and TensorBoard visualizations

Deployed using custom-built Docker containers and AWS ECR/SageMaker integration

Inference pipeline outputs results in annotated frames and a GIF animation

⚙️ Setup & Installation
Please refer to the Setup Instructions provided in the Udacity classroom to configure your SageMaker notebook instance.

Recommended Kernel: conda_tensorflow2_p310
Use manual installs inside notebooks where required (%pip install tensorflow_io sagemaker)

📈 How to Use
Clone this repository to your SageMaker instance:

bash
Copy
Edit
git clone https://github.com/iamaarc/urban-object-detection.git
cd urban-object-detection
Run the notebook: 1_train_model.ipynb

Configure model training settings (model, steps, logs)

Train your model and track progress via TensorBoard

Run the notebook: 2_deploy_model.ipynb

Load the exported model

Run inference on test images

Visualize results (video/GIF output)

📁 Outputs
Trained model checkpoints (/opt/training)

TensorBoard logs (s3://<your-bucket>/logs)

Annotated video frames

Generated GIF of inference output

🧠 Model & Architecture
Model Used: EfficientDet D1
Input Image Resolution: 640x640
Transfer Learning: Pretrained on COCO 2017
Frameworks: TensorFlow 2.x, Object Detection API

📚 References & Resources
📘 TensorFlow Object Detection API Documentation

📘 Training on AWS SageMaker with TensorFlow 2

📘 Waymo Open Dataset

📘 Udacity SDCND - Object Detection Project Guide

👤 Author
Aayush Chugh
LinkedIn | GitHub | Email
