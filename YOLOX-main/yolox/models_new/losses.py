#!/usr/bin/env python
# -*- encoding: utf-8 -*-
# Copyright (c) Megvii Inc. All rights reserved.

import torch
import torch.nn as nn


class alpha_IOUloss(nn.Module):
    def __init__(self, reduction="none", loss_type="cir_iou",alpha=3):
        super(alpha_IOUloss, self).__init__()
        self.reduction = reduction
        self.loss_type = loss_type

        self.alpha = alpha

    def circular_iou_loss(self,pred_boxes, gt_boxes, eps=1e-6):
        """
        Computes Circular IoU (CIoU) for circular objects.
        :param pred_boxes: Predicted bounding boxes (x, y, w, h)
        :param gt_boxes: Ground-truth bounding boxes (x, y, w, h)
        """
        px, py, pw, ph = pred_boxes.unbind(-1)
        gx, gy, gw, gh = gt_boxes.unbind(-1)
    
        # Convert to circle radius (assuming inscribed square method)
        pr = torch.min(pw, ph) / 2
        gr = torch.min(gw, gh) / 2
    
        # Compute center distance
        center_dist = (px - gx) ** 2 + (py - gy) ** 2
    
        # Compute IoU for circles
        intersection = 3.14159 * torch.min(pr, gr) ** 2
        union = 3.14159 * (pr ** 2 + gr ** 2) - intersection
        iou = intersection / (union + eps)
    
        # Compute IoU loss
        loss = 1 - iou
        return loss.mean()
    def shape_iou(self,box1, box2, xywh=True, scale=0 ,eps=1e-7):
        	(x1, y1, w1, h1), (x2, y2, w2, h2) = box1.chunk(4, -1), box2.chunk(4, -1)
        	w1_, h1_, w2_, h2_ = w1 / 2, h1 / 2, w2 / 2, h2 / 2
        	b1_x1, b1_x2, b1_y1, b1_y2 = x1 - w1_, x1 + w1_, y1 - h1_, y1 + h1_
        	b2_x1, b2_x2, b2_y1, b2_y2 = x2 - w2_, x2 + w2_, y2 - h2_, y2 + h2_
	
        	# Intersection area
        	inter = (torch.min(b1_x2, b2_x2) - torch.max(b1_x1, b2_x1)).clamp(0) * \
                (torch.min(b1_y2, b2_y2) - torch.max(b1_y1, b2_y1)).clamp(0)

        	# Union Area
        	union = w1 * h1 + w2 * h2 - inter + eps

        	# IoU
        	iou = inter / union

        	# Shape-Distance    #Shape-Distance    #Shape-Distance    #Shape-Distance    #Shape-Distance    #Shape-Distance    #Shape-Distance
        	ww = 2 * torch.pow(w2, scale) / (torch.pow(w2, scale) + torch.pow(h2, scale))
        	hh = 2 * torch.pow(h2, scale) / (torch.pow(w2, scale) + torch.pow(h2, scale))
        	cw = torch.max(b1_x2, b2_x2) - torch.min(b1_x1, b2_x1)  # convex width
        	ch = torch.max(b1_y2, b2_y2) - torch.min(b1_y1, b2_y1)  # convex height
        	c2 = cw ** 2 + ch ** 2 + eps  # convex diagonal squared
        	center_distance_x = ((b2_x1 + b2_x2 - b1_x1 - b1_x2) ** 2) / 4
        	center_distance_y = ((b2_y1 + b2_y2 - b1_y1 - b1_y2) ** 2) / 4
        	center_distance = hh * center_distance_x + ww * center_distance_y
        	distance = center_distance / c2

        	# Shape-Shape    #Shape-Shape    #Shape-Shape    #Shape-Shape    #Shape-Shape    #Shape-Shape    #Shape-Shape    #Shape-Shape
        	omiga_w = hh * torch.abs(w1 - w2) / torch.max(w1, w2)
        	omiga_h = ww * torch.abs(h1 - h2) / torch.max(h1, h2)
        	shape_cost = torch.pow(1 - torch.exp(-1 * omiga_w), 4) + torch.pow(1 - torch.exp(-1 * omiga_h), 4)

        	# Shape-IoU    #Shape-IoU    #Shape-IoU    #Shape-IoU    #Shape-IoU    #Shape-IoU    #Shape-IoU    #Shape-IoU    #Shape-IoU
        	iou = iou - distance - 0.5 * (shape_cost)
        	return iou  # IoU

    def forward(self, pred, target):
        assert pred.shape[0] == target.shape[0]

        pred = pred.view(-1, 4)
        target = target.view(-1, 4)
        tl = torch.max(
            (pred[:, :2] - pred[:, 2:] / 2), (target[:, :2] - target[:, 2:] / 2)
        )
        br = torch.min(
            (pred[:, :2] + pred[:, 2:] / 2), (target[:, :2] + target[:, 2:] / 2)
        )

        area_p = torch.prod(pred[:, 2:], 1)
        area_g = torch.prod(target[:, 2:], 1)

        en = (tl < br).type(tl.type()).prod(dim=1)
        area_i = torch.prod(br - tl, 1) * en
        area_u = area_p + area_g - area_i
        iou = (area_i) / (area_u + 1e-7)

        if self.loss_type == "iou":
            loss = 1 - iou ** self.alpha   ###############   2>>>>3(a-iou)
        elif self.loss_type == "giou":
            c_tl = torch.min(
                (pred[:, :2] - pred[:, 2:] / 2), (target[:, :2] - target[:, 2:] / 2)
            )
            c_br = torch.max(
                (pred[:, :2] + pred[:, 2:] / 2), (target[:, :2] + target[:, 2:] / 2)
            )
            area_c = torch.prod(c_br - c_tl, 1)
            giou = iou** self.alpha - ((area_c - area_u) / area_c.clamp(1e-16))** self.alpha
            loss = 1 - giou.clamp(min=-1.0, max=1.0)


        elif self.loss_type == "ciou":
            c_tl = torch.min(
                (pred[:, :2] - pred[:, 2:] / 2), (target[:, :2] - target[:, 2:] / 2)
            )
            c_br = torch.max(
                (pred[:, :2] + pred[:, 2:] / 2), (target[:, :2] + target[:, 2:] / 2)
            )
            convex_dis = torch.pow(c_br[:, 0]-c_tl[:, 0], 2) + torch.pow(c_br[:, 1]-c_tl[:,1], 2) + 1e-7 # convex diagonal squared

            center_dis = (torch.pow(pred[:, 0]-target[:, 0], 2) + torch.pow(pred[:, 1]-target[:,1], 2)) # center diagonal squared

            v = (4 / 3.1415926 ** 2) * torch.pow(torch.atan(target[:, 2] / torch.clamp(target[:, 3], min = 1e-7)) - 
                                                torch.atan(pred[:, 2] / torch.clamp(pred[:, 3], min = 1e-7)), 2)
            with torch.no_grad():
                beat = v / (v - iou + 1)
            
            ciou = iou** self.alpha - (center_dis** self.alpha / convex_dis** self.alpha + (beat * v)** self.alpha)
            
            loss = 1 - ciou.clamp(min=-1.0, max=1.0)
        elif self.loss_type == "cir_iou":
            loss = self.circular_iou_loss(pred, target)

        elif self.loss_type == "siou":
            loss = 1 - self.shape_iou(pred, target).clamp(min=-1.0, max=1.0)
	

        if self.reduction == "mean":
            loss = loss.mean()
        elif self.reduction == "sum":
            loss = loss.sum()

        return loss

class IOUloss(nn.Module):
    def __init__(self, reduction="none", loss_type="iou"):
        super(IOUloss, self).__init__()
        self.reduction = reduction
        self.loss_type = loss_type

    def forward(self, pred, target):
        assert pred.shape[0] == target.shape[0]

        pred = pred.view(-1, 4)
        target = target.view(-1, 4)
        tl = torch.max(
            (pred[:, :2] - pred[:, 2:] / 2), (target[:, :2] - target[:, 2:] / 2)
        )
        br = torch.min(
            (pred[:, :2] + pred[:, 2:] / 2), (target[:, :2] + target[:, 2:] / 2)
        )

        area_p = torch.prod(pred[:, 2:], 1)
        area_g = torch.prod(target[:, 2:], 1)

        en = (tl < br).type(tl.type()).prod(dim=1)
        area_i = torch.prod(br - tl, 1) * en
        area_u = area_p + area_g - area_i
        iou = (area_i) / (area_u + 1e-16)

        if self.loss_type == "iou":
            loss = 1 - iou ** 2
        elif self.loss_type == "giou":
            c_tl = torch.min(
                (pred[:, :2] - pred[:, 2:] / 2), (target[:, :2] - target[:, 2:] / 2)
            )
            c_br = torch.max(
                (pred[:, :2] + pred[:, 2:] / 2), (target[:, :2] + target[:, 2:] / 2)
            )
            area_c = torch.prod(c_br - c_tl, 1)
            giou = iou - (area_c - area_u) / area_c.clamp(1e-16)
            loss = 1 - giou.clamp(min=-1.0, max=1.0)

        if self.reduction == "mean":
            loss = loss.mean()
        elif self.reduction == "sum":
            loss = loss.sum()

        return loss
