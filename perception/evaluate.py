import numpy as np
from glob import glob
import csv
import torch
from torchvision import models, transforms
from torch import nn
from PIL import Image
from torchvision import transforms

def write_labels(path, transforms, device):
    files = glob('{}/*/*_image.jpg'.format(path))
    files.sort()
    name = '{}/test_labels_2.csv'.format(path)
    with open(name, 'w') as f:
        writer = csv.writer(f, delimiter=',', lineterminator='\n')
        writer.writerow(['guid/image', 'label'])

        for file in files:
            #note that you need to change "\\" to "/" on mac
            guid = file.split("\\")[-2]
            idx = file.split('\\')[-1].replace('_image.jpg', '')
            image = Image.open(file)
            input_tensor = transforms(image)
            input_batch = input_tensor.unsqueeze(0)
            input_batch = input_batch.to(device)

            with torch.no_grad():
                output = model(input_batch)
            _, label = torch.max(output.data, 1)
            
            writer.writerow(['{}/{}'.format(guid, idx), label[0].item()])
    
    print('Wrote report file `{}`'.format(name))

if __name__ == '__main__':
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = models.resnet101(pretrained=True)
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, 3)
    model.load_state_dict(torch.load("./model_weights_101_nocrop.pth"))
    model.eval()
    print("model loaded!")
    model = model.to(device)

    transforms = transforms.Compose([
        transforms.Resize(224),
        #transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    write_labels("./dataset/test", transforms, device)
