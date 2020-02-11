#! /usr/bin/env python

import numpy as np
import g2o

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super(PoseGraphOptimization, self).__init__()
        # solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super(PoseGraphOptimization, self).set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super(PoseGraphOptimization, self).initialize_optimization()
        super(PoseGraphOptimization, self).optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se2 = g2o.VertexSE2()
        v_se2.set_id(id)
        v_se2.set_estimate(pose)
        v_se2.set_fixed(fixed)
        super(PoseGraphOptimization, self).add_vertex(v_se2)
        # v_se3 = g2o.VertexSE3()
        # v_se3.set_id(id)
        # v_se3.set_estimate(pose)
        # v_se3.set_fixed(fixed)
        # super().add_vertex(v_se3)

    def add_landmark_vertex(self, id, pose, fixed=False):
        v_xy = g2o.VertexPointXY()
        v_xy.set_id(id)
        v_xy.set_estimate(pose)
        v_xy.set_fixed(fixed)
        super(PoseGraphOptimization, self).add_vertex(v_xy)

    def add_edge(self, vertices, measurement,
            information=np.identity(3),
            robust_kernel=None):

        edge = g2o.EdgeSE2()
    
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super(PoseGraphOptimization, self).add_edge(edge)

    def add_landmark_edge(self, vertices,
            robust_kernel=None):

        edge = g2o.EdgeSE2PointXY()
            
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super(PoseGraphOptimization, self).add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()