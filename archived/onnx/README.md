# Archived ONNX experiment

The active perception workflow uses the trained Faster R-CNN PyTorch checkpoint
(`detector.pt`) directly with CUDA.

This directory preserves the earlier ONNX exporter and Helm templates for
historical reference. The ONNX runtime path produced reshape errors when tested
against the live detector, so it is disabled and is not part of the current
deployment or validation workflow.
