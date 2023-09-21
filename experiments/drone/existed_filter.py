import numpy as np
import cvxpy as cp


class ExistedFilter:
    def __init__(self, H_max, h_max, H_xu, h_xu):
        self.f = True
        self.H_max = H_max
        self.h_max = h_max
        self.H_xu = H_xu
        self.h_xu = h_xu
        self.u_pre = [0, 0]  # previous input

        self.pos_ind = H_max[:, -1] > 0
        self.zero_ind = H_max[:, -1] == 0
        self.neg_ind = H_max[:, -1] < 0
        if not np.any(self.pos_ind):
            self.pos_ind = None
        if not np.any(self.neg_ind):
            self.neg_ind = None

    def get_max_alpha(self, x):
        '''
        Compute the maximal alpha given the current state

        Inputs: x (np.Array)--- the state of one subsystem, 3x1
                H_max, h_max (np.Array) --- [H_max, h_max] is the H-representation 
                                of the maximal RCIS for the alpha dynmaics
        Return: alpha --- the maximal alpha at x
        '''
        A = self.H_max[:, -1]
        b = self.h_max - self.H_max[:, 0:-1] @ x
        alpha_max = np.inf
        alpha_min = -np.inf
        if self.pos_ind is not None:
            alpha_max = np.min(b[self.pos_ind].squeeze() / A[self.pos_ind])
        if self.neg_ind is not None:
            alpha_min = np.max(b[self.neg_ind].squeeze() / A[self.neg_ind])

        if (alpha_max < alpha_min) or (np.any(b[self.zero_ind] < -1e-6)):
            alpha_max = -np.inf
            alpha_min = - np.inf

        return alpha_max, alpha_min

    def qp_1d(self, x, u_ref):
        '''
        Solve the QP program in one subsystem.
        Inputs: x (np.array) ---  the states of one subsystem (i.e. x, vx, alpha)
                u_ref (float) --- the reference input
        Return: u_corr (float)--- the supervised input
        '''
        u = cp.Variable(3)

        H = self.H_xu[:, 4:]
        h = self.h_xu - self.H_xu[:, 0:4] @ x
        # import pdb; pdb.set_trace()
        prob = cp.Problem(cp.Minimize((u[0] - u_ref) ** 2),
                          [H @ u <= h.squeeze()])
        prob.solve(cp.PROXQP)
        return u.value[0]

    def eval(self, xy, u_ref):
        '''
        Project the reference input to the admissible input set at x.
        Inputs: xy (list) --- the states of the 4d system (x, y, vx, vy)
                u_ref (list) --- the reference input, 1x2
        Return: u (list) --- the supervised input, 1x2
        '''
        x = np.array([xy[0], xy[2], self.u_pre[0]]).reshape((-1, 1))
        y = np.array([xy[1], xy[3], self.u_pre[1]]).reshape((-1, 1))
        alpha_x, _ = self.get_max_alpha(x)
        alpha_y, _ = self.get_max_alpha(y)

        if alpha_x < 1:
            ux = u_ref[0]
            print('alpha x is -inf')
        else:
            print('alpha x is %f' % (alpha_x))
            ux = self.qp_1d(np.vstack((x, alpha_x)), u_ref[0])

        if alpha_y < 1:
            uy = u_ref[1]
            print('alpha y is -inf')
        else:
            print('alpha y is %f' % (alpha_y))
            uy = self.qp_1d(np.vstack((y, alpha_y)), u_ref[1])

        self.u_pre = [ux, uy]
        return [ux, uy], [alpha_x, alpha_y]


if __name__ == '__main__':
    from scipy.io import loadmat
    import time

    data = loadmat('alpha_filter_py.mat')
    alpha_filter = ExistedFilter(data['H_max'], data['h_max'], data['H_xu'], data['h_xu'])
    x = [-0.061786048114299774, -0.008449121378362179, -0.010084494017064571, 0.003397843334823847]
    u_ref = [0.053372868658753146, 0.005272841065899434]
    t = time.time()
    print(alpha_filter.eval(x, u_ref))
    print(time.time() - t)
