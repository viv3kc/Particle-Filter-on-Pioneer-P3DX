import numpy as np
import random


def cluster_points(pose_points, mu, poses):
    idx = 0
    clusters = {}
    pose_clusters = {}
    for x in pose_points:
        bestmukey = min([(i[0], np.linalg.norm(x - mu[i[0]]))
                         for i in enumerate(mu)], key=lambda t: t[1])[0]
        try:
            clusters[bestmukey].append(x)
            pose_clusters[bestmukey].append(poses[idx])
        except KeyError:
            clusters[bestmukey] = [x]
            pose_clusters[bestmukey] = [poses[idx]]
        idx += 1
    return clusters, pose_clusters


def eval_centers(clusters):
    newmu = []
    keys = sorted(clusters.keys())
    for k in keys:
        newmu.append(np.mean(clusters[k], axis=0))
    return newmu


def are_the_same(mu, oldmu):
    return set([tuple(a) for a in mu]) == set([tuple(a) for a in oldmu])


def get_biggest_cluster(poses, K):
    # Initialize to K random centers
    pose_points = np.array([(pose.position.x, pose.position.y) for pose in poses])
    oldmu = random.sample(pose_points, K)
    mu = random.sample(pose_points, K)
    pose_clusters = {}  # Poses clustered around a mean value
    while not are_the_same(mu, oldmu):
        oldmu = mu
        # Assign all points in pose_points to clusters
        (clusters, pose_clusters) = cluster_points(pose_points, mu, poses)
        # Reevaluate centers
        mu = eval_centers(clusters)

    # Get the cluster containing the biggest number of poses
    max_len = 0
    big_cluster = []
    for key in pose_clusters.keys():
        if max_len < len(pose_clusters[key]):
            max_len = len(pose_clusters[key])
            big_cluster = pose_clusters[key]
    return big_cluster
