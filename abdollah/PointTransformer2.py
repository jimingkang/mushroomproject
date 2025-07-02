# models/point_transformer.py

import torch
import torch.nn as nn
import torch.nn.functional as F
import lightning as pl
from utils import rotloss, farthest_point_sample, index_points, square_distance
from rotation_conversions import matrix_to_euler_angles,matrix_to_quaternion,matrix_to_rotation_6d, rotation_6d_to_matrix, quaternion_to_matrix, axis_angle_to_matrix, euler_angles_to_matrix
import matplotlib.pyplot as plt
import time
import numpy as np

# Include the necessary utility functions and classes from the provided code
# Ensure that Local_op, SA_Layer, and StackedAttention classes are included

def sample_and_group(npoint, nsample, xyz, points):
    B, N, C = xyz.shape
    S = npoint 
    
    fps_idx = farthest_point_sample(xyz, npoint)  # [B, npoint]

    new_xyz = index_points(xyz, fps_idx) 
    new_points = index_points(points, fps_idx)

    dists = square_distance(new_xyz, xyz)  # B x npoint x N
    idx = dists.argsort()[:, :, :nsample]  # B x npoint x K

    grouped_points = index_points(points, idx)
    grouped_points_norm = grouped_points - new_points.view(B, S, 1, -1)
    new_points = torch.cat([grouped_points_norm, new_points.view(B, S, 1, -1).repeat(1, 1, nsample, 1)], dim=-1)
    return new_xyz, new_points

class Local_op(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.conv1 = nn.Conv1d(in_channels, out_channels, kernel_size=1, bias=False)
        self.conv2 = nn.Conv1d(out_channels, out_channels, kernel_size=1, bias=False)
        self.bn1 = nn.BatchNorm1d(out_channels)
        self.bn2 = nn.BatchNorm1d(out_channels)
        self.relu = nn.ReLU()

    def forward(self, x):
        b, n, s, d = x.size()  # torch.Size([B, npoint, nsample, in_channels])
        x = x.permute(0, 1, 3, 2)
        x = x.reshape(-1, d, s)
        batch_size, _, N = x.size()
        x = self.relu(self.bn1(self.conv1(x)))  # B, D, N
        x = self.relu(self.bn2(self.conv2(x)))  # B, D, N
        x = torch.max(x, 2)[0]
        x = x.view(b, n, -1).permute(0, 2, 1)
        return x

class SA_Layer(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.q_conv = nn.Conv1d(channels, channels // 4, 1, bias=False)
        self.k_conv = nn.Conv1d(channels, channels // 4, 1, bias=False)
        self.q_conv.weight = self.k_conv.weight 
        self.v_conv = nn.Conv1d(channels, channels, 1)
        self.trans_conv = nn.Conv1d(channels, channels, 1)
        self.after_norm = nn.BatchNorm1d(channels)
        self.act = nn.ReLU()
        self.softmax = nn.Softmax(dim=-1)

    def forward(self, x):
        x_q = self.q_conv(x).permute(0, 2, 1)  # b, n, c 
        x_k = self.k_conv(x)  # b, c, n        
        x_v = self.v_conv(x)
        energy = x_q @ x_k  # b, n, n 
        attention = self.softmax(energy)
        attention = attention / (1e-9 + attention.sum(dim=1, keepdims=True))
        x_r = x_v @ attention  # b, c, n 
        x_r = self.act(self.after_norm(self.trans_conv(x - x_r)))
        x = x + x_r
        return x

class StackedAttention(nn.Module):
    def __init__(self, channels=256):
        super().__init__()
        self.conv1 = nn.Conv1d(channels, channels, kernel_size=1, bias=False)
        self.conv2 = nn.Conv1d(channels, channels, kernel_size=1, bias=False)

        self.bn1 = nn.BatchNorm1d(channels)
        self.bn2 = nn.BatchNorm1d(channels)

        self.sa1 = SA_Layer(channels)
        self.sa2 = SA_Layer(channels)
        self.sa3 = SA_Layer(channels)
        self.sa4 = SA_Layer(channels)

        self.relu = nn.ReLU()
        
    def forward(self, x):
        batch_size, _, N = x.size()

        x = self.relu(self.bn1(self.conv1(x)))  # B, D, N
        x = self.relu(self.bn2(self.conv2(x)))

        x1 = self.sa1(x)
        x2 = self.sa2(x1)
        x3 = self.sa3(x2)
        x4 = self.sa4(x3)
        
        x = torch.cat((x1, x2, x3, x4), dim=1)

        return x

class PointTransformerModel(pl.LightningModule):
    def __init__(self, num_predictions=100, lr=1e-4, rgb=False, rotation = "6D",):
        super(PointTransformerModel, self).__init__()
        self.num_predictions = num_predictions
        self.lr = lr
        self.rgb = rgb
        d_points = 6 if self.rgb else 3  # Changed from 6 to 3 (XYZ only)
        self.rotation = rotation

        self.conv1 = nn.Conv1d(d_points, 64, kernel_size=1, bias=False)
        self.conv2 = nn.Conv1d(64, 64, kernel_size=1, bias=False)
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(64)
        self.gather_local_0 = Local_op(in_channels=128, out_channels=128)
        self.gather_local_1 = Local_op(in_channels=256, out_channels=256)
        self.pt_last = StackedAttention()

        self.relu = nn.ReLU()
        self.conv_fuse = nn.Sequential(
            nn.Conv1d(1280, 1024, kernel_size=1, bias=False),
            nn.BatchNorm1d(1024),
            nn.LeakyReLU(negative_slope=0.2)
        )

        # Output layers for predictions
        # For each prediction: rotations(6)        
        self.rotation_dims = 0
        if self.rotation == "6D":
            self.rotation_dims = 6  # Predict rotations (6D representation)
        elif self.rotation == "9D":
            self.rotation_dims = 9  # Predict rotations (9D representation)
        elif self.rotation == "quat":
            self.rotation_dims = 4  # Predict rotations (quaternion representation)
        elif self.rotation == "euler":
            self.rotation_dims = 3  # Predict rotations (Euler angles representation)


        output_dim = self.num_predictions * self.rotation_dims

        self.linear1 = nn.Linear(1024, 512, bias=False)
        self.bn6 = nn.BatchNorm1d(512)
        self.dp1 = nn.Dropout(p=0.5)
        self.linear2 = nn.Linear(512, 256)
        self.bn7 = nn.BatchNorm1d(256)
        self.dp2 = nn.Dropout(p=0.5)
        self.linear3 = nn.Linear(256, output_dim)

    def forward(self, x):
        # x is of shape (batch_size, num_points, num_features)
        xyz = x[..., :6] if self.rgb else x[..., :3]
        x = x.permute(0, 2, 1)  # Shape: (batch_size, num_features, num_points)
        batch_size, _, _ = x.size()
        x = self.relu(self.bn1(self.conv1(x)))  # B, D, N
        x = self.relu(self.bn2(self.conv2(x)))  # B, D, N
        x = x.permute(0, 2, 1)
        new_xyz, new_feature = sample_and_group(npoint=512, nsample=32, xyz=xyz, points=x)
        feature_0 = self.gather_local_0(new_feature)
        feature = feature_0.permute(0, 2, 1)
        new_xyz, new_feature = sample_and_group(npoint=256, nsample=32, xyz=new_xyz, points=feature)
        feature_1 = self.gather_local_1(new_feature)

        x = self.pt_last(feature_1)
        x = torch.cat([x, feature_1], dim=1)
        x = self.conv_fuse(x)
        x = torch.max(x, 2)[0]  # Global max pooling
        x = x.view(batch_size, -1)

        x = self.relu(self.bn6(self.linear1(x)))
        x = self.dp1(x)
        x = self.relu(self.bn7(self.linear2(x)))
        x = self.dp2(x)
        x = self.linear3(x)  # Output layer

        # Reshape outputs
        x = x.view(-1, self.num_predictions, self.rotation_dims)  # (batch_size, num_predictions, output_dim_per_prediction)

        rotations = x  # (batch_size, num_predictions, 6)

        # Normalize rotations
        rotations = rotations / rotations.norm(dim=-1, keepdim=True)

        return rotations

    def training_step(self, batch, batch_idx):
        point_clouds, gt_rotations, scale = batch
        if self.rotation == "6D":
            gt_rotations = matrix_to_rotation_6d(gt_rotations)
        elif self.rotation == "quat":
            gt_rotations = matrix_to_quaternion(gt_rotations)
        elif self.rotation == "euler":
            gt_rotations = matrix_to_euler_angles(gt_rotations, convention="XYZ")
        else:
            pass

        # Move data to the appropriate device
        point_clouds = point_clouds.to(self.device)
        gt_rotations = gt_rotations.to(self.device)

        # Forward pass
        rotations_pred = self(point_clouds)  # (batch_size, num_predictions, 6)

        # For simplicity, we assume one prediction per sample (num_predictions=1)
        # and match the first prediction with the ground truth
        rotations_pred = rotations_pred[:, 0, :]  # (batch_size, 6)
        gt_rotations = gt_rotations  # (batch_size, 6)

        # Compute rotation loss
        rotations_loss = rotloss(rotations_pred, gt_rotations, rotation=self.rotation)

        self.log('train_loss', rotations_loss, sync_dist=True, prog_bar=True)
        return rotations_loss

    def validation_step(self, batch, batch_idx):
        point_clouds, gt_rotations, scale = batch
        if self.rotation == "6D":
            gt_rotations = matrix_to_rotation_6d(gt_rotations)
        elif self.rotation == "quat":
            gt_rotations = matrix_to_quaternion(gt_rotations)
        elif self.rotation == "euler":
            gt_rotations = matrix_to_euler_angles(gt_rotations, convention="XYZ")
        else:
            pass

        # Move data to the appropriate device
        point_clouds = point_clouds.to(self.device)
        gt_rotations = gt_rotations.to(self.device)

        # Forward pass
        rotations_pred = self(point_clouds)  # (batch_size, num_predictions, 6)

        # Match the first prediction with the ground truth
        rotations_pred = rotations_pred[:, 0, :]  # (batch_size, 6)

        # Compute rotation loss
        rotations_loss = rotloss(rotations_pred, gt_rotations, rotation = self.rotation)

        self.log('val_loss', rotations_loss, sync_dist=True, prog_bar=True)
        return rotations_loss
 
    def convert_to_rotation_matrix(self, rotations_pred, rotation_type):
        if rotation_type == '6D':
            # rotations_pred: (batch_size, 6)
            # Convert 6D rotations to rotation matrices
            pred_rotation_matrices = rotation_6d_to_matrix(rotations_pred)
        elif rotation_type == 'quat':
            # rotations_pred: (batch_size, 4)
            # Convert quaternions to rotation matrices
            pred_rotation_matrices = quaternion_to_matrix(rotations_pred)
        elif rotation_type == 'euler':
            # rotations_pred: (batch_size, 3)
            # Convert Euler angles to rotation matrices
            pred_rotation_matrices = euler_angles_to_matrix(rotations_pred, convention='XYZ')
        elif rotation_type == '9D':
            # rotations_pred: (batch_size, 3)
            # Convert Euler angles to rotation matrices
            pred_rotation_matrices = rotations_pred.reshape(-1,3,3)
        else:
            raise ValueError(f'Unknown rotation type: {rotation_type}')
        return pred_rotation_matrices

    def on_test_epoch_start(self):
        # Initialize lists to store outputs
        self.test_total_losses = []
        self.test_rotation_diffs = []
        self.test_sizes = []
        self.test_inference_times = []
        self.preds = []
        self.gts = []

    def test_step(self, batch, batch_idx):
        point_clouds, gt_rotations, dimensions = batch

        # Move data to the appropriate device
        point_clouds = point_clouds.to(self.device)
        gt_rotations = gt_rotations.to(self.device)  # (batch_size, 3, 3)
        dimensions = dimensions.to(self.device)

        # Convert ground truth rotations to the appropriate representation
        if self.rotation == "6D":
            gt_rotations_rep = matrix_to_rotation_6d(gt_rotations)
        elif self.rotation == "quat":
            gt_rotations_rep = matrix_to_quaternion(gt_rotations)
        elif self.rotation == "euler":
            gt_rotations_rep = matrix_to_euler_angles(gt_rotations, convention="XYZ")
        else:
            gt_rotations_rep = gt_rotations  # If rotation is already in matrix form


        # Measure inference time
        start_time = time.time()

        # Forward pass
        rotations_pred = self(point_clouds)  # (batch_size, num_predictions, rotation_dims)

        inference_time = time.time() - start_time

        # Match the first prediction with the ground truth
        rotations_pred = rotations_pred[:, 0, :]  # (batch_size, rotation_dims)

        # Convert rotations_pred to rotation matrices
        pred_rotation_matrices = self.convert_to_rotation_matrix(rotations_pred, self.rotation)

        # Compute relative rotations (rotation differences)
        rel_rotations = pred_rotation_matrices.transpose(1, 2) @ gt_rotations

        # Convert relative rotations to Euler angles (roll, pitch, yaw differences)
        rotation_diffs = matrix_to_euler_angles(rel_rotations, convention='XYZ')  # (batch_size, 3)

        # Compute total loss
        rotations_loss = rotloss(rotations_pred, gt_rotations_rep, rotation=self.rotation)

        # Compute per-sample losses (L2 norm of rotation differences)
        rotations_loss_per_sample = torch.norm(rotation_diffs, dim=1)  # (batch_size,)

        # Compute mushroom sizes using dimensions
        sizes = torch.sqrt(dimensions[:, 0] ** 2 + dimensions[:, 1] ** 2)  # (batch_size,)

        # Log total loss value
        self.log('test_loss', rotations_loss, sync_dist=True, prog_bar=True)

        # Store outputs in instance attributes
        self.test_total_losses.append(rotations_loss.detach().cpu())
        self.test_rotation_diffs.append(rotation_diffs.detach().cpu())
        self.test_sizes.append(sizes.detach().cpu())
        self.test_inference_times.append(inference_time / point_clouds.size(0))  # Average per sample
        self.preds.append(pred_rotation_matrices)
        self.gts.append(gt_rotations)

        rotation_diffs = torch.cat(self.test_rotation_diffs, dim=0)
        rotation_diffs_deg = np.abs(rotation_diffs.numpy()) * (180.0 / np.pi)
        print(np.mean(rotation_diffs_deg))
        # Return None (since outputs are stored in instance attributes)
        return None

    def on_test_epoch_end(self):
        # Aggregate results
        total_losses = torch.stack(self.test_total_losses)  # (num_batches,)
        rotation_diffs = torch.cat(self.test_rotation_diffs, dim=0)  # (total_samples, 3)
        preds = torch.cat(self.preds, dim=0)
        gts = torch.cat(self.gts, dim=0)

        sizes = torch.cat(self.test_sizes, dim=0)  # (total_samples,)
        avg_inference_time = np.mean(self.test_inference_times)

        # Log the average total loss
        avg_total_loss = total_losses.mean()
        self.log('avg_test_loss', avg_total_loss, sync_dist=True, prog_bar=True)

        # Convert rotation differences from radians to degrees and take absolute value
        rotation_diffs_deg = np.abs(rotation_diffs.numpy()) * (180.0 / np.pi)
        print(f"############### FINAL ROTATION DEGREE DIFF: {np.mean(rotation_diffs_deg,axis=0)}")
        print(f"############### FINAL ROTATION COSINE DIFF: {torch.mean(torch.nn.CosineSimilarity(dim=0)(preds,gts),axis=0)}")

        plt.rcParams.update({
            'axes.titlesize': 26,     # Title font size
            'axes.labelsize': 26,     # X and Y label font size
            'xtick.labelsize': 20,    # X-axis tick font size
            'ytick.labelsize': 20,    # Y-axis tick font size
            'legend.fontsize': 20    # Legend font size (if used)
        })
        # Plot histograms for absolute roll, pitch, yaw differences
        angle_names = ['Roll', 'Pitch', 'Yaw']
        for i in range(3):
            plt.figure()
            plt.hist(rotation_diffs_deg[:, i], bins=100, color='blue', alpha=0.7, log=True)
            plt.title(f'Absolute {angle_names[i]} Differences')
            plt.xlabel(f'Difference (degrees)')
            plt.ylabel('Frequency')
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(f'rotation_difference_{angle_names[i].lower()}.pdf')
            plt.close()

        # Plot rotation differences vs. mushroom sizes for each angle separately
        sizes_np = sizes.numpy()  # (total_samples,)

        for i in range(3):
            plt.figure()
            plt.scatter(sizes_np, rotation_diffs_deg[:, i], alpha=0.7)
            plt.title(f'{angle_names[i]} Difference vs. Size')
            plt.xlabel('Mushroom Size (pixels)')
            plt.ylabel(f'Difference (degrees)')
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(f'rotation_difference_vs_size_{angle_names[i].lower()}.pdf')
            plt.close()

        # Report the average inference time per single sample
        print(f'Average inference time per sample: {avg_inference_time:.6f} seconds')

        # Optionally, you can store the aggregated results as instance attributes
        self.avg_test_loss = avg_total_loss
        self.rotation_diffs = rotation_diffs
        self.sizes = sizes
        self.avg_inference_time = avg_inference_time

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.lr)
        return optimizer
