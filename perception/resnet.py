import os
import torch
from torch.utils.data import Dataset
from torchvision import models, transforms
import torch.optim as optim
from torch.optim import lr_scheduler
from torchvision.transforms import ToTensor
import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image
from torch.utils.data import DataLoader
from torch.utils.data import random_split
from torch import nn
import time
import copy
import ssl

#Custom Dataset Definition
class VehicleDataset(Dataset):
	def __init__(self, annotations_file, img_dir, transform=None):
		self.img_labels = pd.read_csv(annotations_file)
		self.img_dir = img_dir
		self.transform = transform
		
	def __len__(self):
		return len(self.img_labels)
		
	def __getitem__(self, idx):
		img_labels = str(self.img_labels.iloc[idx, 0]) + "_image.jpg"
		img_path = os.path.join(self.img_dir, img_labels)
		image = Image.open(img_path)
		label = self.img_labels.iloc[idx, 1]
		if self.transform:
			image = self.transform(image)
		return image, label

def set_parameter_requires_grad(model):
	for name, param in model.named_parameters():
		if(name[5] == "3" or name[5] == "4" or name[0:2] == "fc"):
			param.requires_grad = True
		else:
			param.requires_grad = False


#Function for model training
def train_model(model, train_dataloader, val_dataloader, device, criterion, optimizer, scheduler, num_epochs=25):
	since = time.time()
	best_model_wts = copy.deepcopy(model.state_dict())
	best_acc = 0.0
	
	for epoch in range(num_epochs):
		print('Epoch {}/{}'.format(epoch, num_epochs - 1))
		print('-' * 10)

        # Each epoch has a training and validation phase
		for phase in ['train', 'val']:
			if phase == 'train':
				model.train()  # Set model to training mode
				dataloaders = train_dataloader
			else:
				model.eval()   # Set model to evaluate mode
				dataloaders = val_dataloader
			
			running_loss = 0.0
			running_corrects = 0

            # Iterate over data.
			for inputs, labels in dataloaders:
				inputs = inputs.to(device)
				labels = labels.to(device)

                # zero the parameter gradients
				optimizer.zero_grad()

                # forward
                # track history if only in train
				with torch.set_grad_enabled(phase == 'train'):
					outputs = model(inputs)
					_, preds = torch.max(outputs, 1)
					loss = criterion(outputs, labels)

                    # backward + optimize only if in training phase
					if phase == 'train':
						loss.backward()
						optimizer.step()

                # statistics
				running_loss += loss.item() * inputs.size(0)
				running_corrects += torch.sum(preds == labels.data)
				torch.cuda.empty_cache()

			if phase == 'train':
				scheduler.step()
				
			epoch_loss = running_loss / len(dataloaders)
			epoch_acc = running_corrects.double() / len(dataloaders)
			
			print('{} Loss: {:.4f} Acc: {:.4f}'.format(phase, epoch_loss, epoch_acc))

            # deep copy the model
			if phase == 'val' and epoch_acc > best_acc:
				best_acc = epoch_acc
				best_model_wts = copy.deepcopy(model.state_dict())
				
		print()
	time_elapsed = time.time() - since
	print('Training complete in {:.0f}m {:.0f}s'.format(time_elapsed // 60, time_elapsed % 60))
	print('Best val Acc: {:4f}'.format(best_acc))

    # load best model weights
	model.load_state_dict(best_model_wts)
	return model


if __name__ == '__main__':
	'''
	device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
	model_ft = models.resnet101(pretrained=True)
	num_ftrs = model_ft.fc.in_features
	model_ft.fc = nn.Linear(num_ftrs, 3)
	model_ft = model_ft.to(device)
	set_parameter_requires_grad(model_ft)
	for name, param in  model_ft.named_parameters():
		print(name, param.requires_grad)
	'''
	#dataset
	transforms = transforms.Compose([
		transforms.Resize(224),
		#transforms.CenterCrop(224),
		transforms.ToTensor(),
		transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
	])
	vehicle_dataset = VehicleDataset("./dataset/trainval/labels.csv", "./dataset/trainval/", transforms)
	train, val = random_split(vehicle_dataset, [7572, 1])
	train_dataloader = DataLoader(train, batch_size=64, shuffle=True)
	val_dataloader = DataLoader(val, batch_size=64, shuffle=True)

	#model parameters
	device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
	ssl._create_default_https_context = ssl._create_unverified_context
	model_ft = models.resnet101(pretrained=True)
	num_ftrs = model_ft.fc.in_features
	model_ft.fc = nn.Linear(num_ftrs, 3)
	model_ft = model_ft.to(device)
	set_parameter_requires_grad(model_ft)
	criterion = nn.CrossEntropyLoss()


	optimizer_ft = optim.SGD(model_ft.parameters(), lr=0.001, momentum=0.9)
	exp_lr_scheduler = lr_scheduler.StepLR(optimizer_ft, step_size=7, gamma=0.1)

	#training
	torch.cuda.empty_cache()
	model_ft = train_model(model_ft, train_dataloader, val_dataloader, device, criterion, optimizer_ft, exp_lr_scheduler,num_epochs=15)
	torch.save(model_ft.state_dict(), './model_weights_101_nocrop_64.pth')
	
