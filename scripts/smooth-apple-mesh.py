#!/usr/bin/env python3
"""Generate the apple visual from the supplied OBJ (requires NumPy and SciPy)."""
from pathlib import Path
import numpy as np
from scipy.sparse import coo_matrix, diags


def smooth(source, destination):
    vertices, faces = [], []
    for line in source.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append([float(v) for v in fields[1:4]])
        elif fields[0] == 'f':
            indices = [int(v.split('/')[0]) - 1 for v in fields[1:]]
            faces.extend([indices[0], indices[i], indices[i + 1]]
                         for i in range(1, len(indices) - 1))
    original = np.asarray(vertices)
    # Weld positional duplicates so UV/material seams do not split the surface.
    positions, inverse = np.unique(np.round(original, 9), axis=0, return_inverse=True)
    triangles = inverse[np.asarray(faces)]
    triangles = triangles[np.logical_and.reduce([
        triangles[:, 0] != triangles[:, 1], triangles[:, 1] != triangles[:, 2],
        triangles[:, 2] != triangles[:, 0]])]
    edges = np.vstack([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    edges = np.unique(np.vstack([edges, edges[:, ::-1]]), axis=0)
    adjacency = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                           shape=(len(positions), len(positions))).tocsr()
    degree = np.asarray(adjacency.sum(axis=1)).ravel()
    average = diags(1 / np.maximum(degree, 1)) @ adjacency
    active = degree > 0
    for _ in range(12):
        # Taubin's alternating passes reduce noise without simple Laplacian shrinkage.
        for factor in (0.5, -0.53):
            positions[active] += factor * (average @ positions - positions)[active]
    # Preserve the original dimensions and center used by the collision model.
    lower, upper = original.min(axis=0), original.max(axis=0)
    positions = ((positions - positions.min(axis=0)) / np.ptp(positions, axis=0)
                 * (upper - lower) + lower)
    a, b, c = (positions[triangles[:, i]] for i in range(3))
    face_normals = np.cross(b - a, c - a)
    normals = np.zeros_like(positions)
    for i in range(3):
        np.add.at(normals, triangles[:, i], face_normals)
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths == 0):
        raise ValueError('Cannot compute normals for unused or degenerate vertices')
    normals /= lengths[:, None]
    with destination.open('w') as stream:
        stream.write('# Smoothed supplied apple geometry; materials assigned by SDF.\ns 1\n')
        for vertex in positions:
            stream.write('v ' + ' '.join(f'{x:.10g}' for x in vertex) + '\n')
        for normal in normals:
            stream.write('vn ' + ' '.join(f'{x:.10g}' for x in normal) + '\n')
        for triangle in triangles + 1:
            stream.write('f ' + ' '.join(f'{i}//{i}' for i in triangle) + '\n')
    assert np.isfinite(positions).all() and np.isfinite(normals).all()
    print(f'Wrote {destination}: {len(positions)} vertices, {len(triangles)} triangles')


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    meshes = root / 'src/roboarm_urdf_description/models/ycb_apple/meshes'
    smooth(meshes / 'textured.obj', meshes / 'apple_smooth.obj')
