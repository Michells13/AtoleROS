"""Mask R-CNN (torchvision maskrcnn_resnet50_fpn) con los pesos entrenados de Config.xml.

Mismo modelo y postproceso que PozoleV3 (devices/inference.py): entrada RGB en [0, 1] sin
redimensionar (torchvision normaliza y escala internamente), clases del perfil, score ≥ umbral,
máscara > 0.5 y orden por score descendente.
"""
import time
from pathlib import Path

import numpy as np

# Perfiles de clases (num_classes incluye el fondo). Los nombres coinciden con PozoleV3.
PROFILES = {
    'itabuna': {'num_classes': 2, 'names': {1: 'pod'}},
    'pollux': {'num_classes': 2, 'names': {1: 'pod'}},
    'pods': {'num_classes': 5, 'names': {1: 'Ripe', 2: 'Unripe', 3: 'Trunk', 4: 'UnknownPod'}},
    'room': {'num_classes': 2, 'names': {1: 'ripe'}},
}


class MaskRCNN:

    def __init__(self, path, profile, score_thresh, exclude=(), device='cuda'):
        if profile not in PROFILES:
            raise ValueError(f'perfil de modelo desconocido "{profile}" (opciones: {", ".join(PROFILES)})')
        if not path or not Path(path).is_file():
            raise FileNotFoundError(f'no existe el modelo {path}')
        self.path, self.profile, self.score_thresh = str(path), profile, float(score_thresh)
        self.names = PROFILES[profile]['names']
        excluded = {e.strip().lower() for e in exclude if e.strip()}
        self.active = {i for i, n in self.names.items() if n.lower() not in excluded}
        self.device_name = device
        self.model = None

    def key(self):
        return (self.path, self.profile, self.score_thresh, tuple(sorted(self.active)))

    def load(self):
        import torch
        import torchvision
        device = torch.device(self.device_name if torch.cuda.is_available() else 'cpu')
        # Misma arquitectura que maskrcnn_resnet50_fpn(weights=None) con el backbone por defecto (como
        # PozoleV3): FrozenBatchNorm2d y 3 capas entrenables. Con weights_backbone=None torchvision
        # usaría BatchNorm2d, que da scores y máscaras ligeramente distintos. Se construye a mano para
        # no depender de descargar los pesos de ImageNet, que el checkpoint sobrescribe de todas formas.
        from torchvision.models.detection.backbone_utils import _resnet_fpn_extractor
        from torchvision.ops.misc import FrozenBatchNorm2d
        backbone = _resnet_fpn_extractor(torchvision.models.resnet50(weights=None, norm_layer=FrozenBatchNorm2d), 3)
        model = torchvision.models.detection.MaskRCNN(backbone, num_classes=PROFILES[self.profile]['num_classes'])
        state = torch.load(self.path, map_location=device)
        if isinstance(state, dict) and 'model' in state:
            state = state['model']
        model.load_state_dict(state, strict=False)      # strict=False como PozoleV3
        self.model, self.device = model.eval().to(device), device

    def run(self, bgr):
        """Lista de dicts {class_id, class_name, score, bbox [x, y, w, h], mask uint8 H×W (0/1)}."""
        import torch
        if self.model is None:
            self.load()
        t0 = time.perf_counter()
        rgb = np.ascontiguousarray(bgr[..., ::-1])
        # /255 en CPU y luego a la GPU, como PozoleV3: en la GPU la división no es bit a bit igual y
        # cambia algún píxel de las máscaras.
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float().div(255.0).unsqueeze(0).to(self.device)
        with torch.no_grad():
            pred = self.model(tensor)[0]
        labels, scores = pred['labels'].cpu().tolist(), pred['scores'].cpu().tolist()
        boxes, masks = pred['boxes'].cpu().tolist(), pred['masks']
        dets = []
        for i, (cls, score) in enumerate(zip(labels, scores)):
            if cls not in self.active or score < self.score_thresh:
                continue
            x1, y1, x2, y2 = (int(v) for v in boxes[i])
            dets.append({'class_id': cls, 'class_name': self.names.get(cls, f'class_{cls}'), 'score': float(score),
                         'bbox': [x1, y1, max(1, x2 - x1), max(1, y2 - y1)],
                         'mask': (masks[i, 0] > 0.5).to(torch.uint8).cpu().numpy()})
        dets.sort(key=lambda d: d['score'], reverse=True)
        if self.device.type == 'cuda':
            torch.cuda.synchronize()
        return dets, (time.perf_counter() - t0) * 1000.0
