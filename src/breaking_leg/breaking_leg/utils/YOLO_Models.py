import torch
import torchvision
import cv2
import numpy
torch._C._jit_override_can_fuse_on_gpu(False)

class YOLO:
    def __init__(self, path, img_size=(640, 640), device='cuda'):
        """
        python module for loading torchscript YOLO model.

        path: path to torchscipt model.
        img_size: image size of torchscript model.
        device: device base of torchscript model.

        by: Tran Dang An from FPT University.
        """
        self.path = path
        self.img_size = img_size
        self.device = device
        self.model = torch.jit.load(self.path).to(self.device)
        self.model

    def predict(self, x):
        """
        x: image RGB.

        return prediction.
        prediction:
            [:4] bounding box.
            [4] confidence.
            [5] class.
        """
        x = x.copy()
        x = cv2.resize(x, self.img_size)
        x = torch.tensor(x).permute(2, 0, 1).to(torch.float32)
        x = torch.unsqueeze(x, dim=0)/255
        x = x.to(self.device)
        pred = self.model(x)[0]
        idx = torch.argmax(torch.sum(pred, dim=-1))
        pred = pred[:idx]
        pred[:, :4] = torch.round(pred[:, :4])
        return pred.detach().cpu().numpy()
        
    def draw(self, img, out):
        """
        draw output on image.

        img: image.
        out: output type of forward function.
        """
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 1
        fontColor              = (255,255,255)
        thickness              = 1
        lineType               = 2
        img = img.copy()
        h, w, _ = img.shape
        for i in range(len(out)):
            bbox, conf, clss = out[i][:4], out[i][4], out[i][5]
            x1, y1, x2, y2 = bbox/640
            x1, y1, x2, y2 = int(x1 * w), int(y1 * h), int(x2 * w), int(y2 * h)
            img = cv2.rectangle(img, (x1, y1), (x2, y2), thickness=2, color=(0, 255, 0))
            if clss != 0:
                continue
            img = cv2.putText(img,f'person {conf: .2f}', 
                        (x1, y2), 
                        font, 
                        fontScale,
                        fontColor,
                        thickness,
                        lineType)
        return img