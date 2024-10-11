from policy import *
from cost import *
from fk_num import *
from fk_sym_gen import *
from math import pi

from torch.profiler import record_function
import asyncio
torch.set_printoptions(linewidth=200, sci_mode=False)


@torch.jit.script
def get_mindist(all_links, obs):
    mindists = dist_tens(all_links, obs)
    # calculate repulsions using fk_sym_gen
    idx_obs_closest = mindists[:, 1].to(torch.long).unsqueeze(1)
    idx_links_closest = mindists[:, 2].to(torch.long).unsqueeze(1)
    idx_pts_closest = mindists[:, 3].to(torch.long).unsqueeze(1)
    return mindists[:, 0], idx_obs_closest, idx_links_closest, idx_pts_closest

class MPPI:
    def __init__(self, q0: torch.Tensor, qf: torch.Tensor, dh_params: torch.Tensor, obs: torch.Tensor, dt: float,
             dt_H: int, N_traj: int, DS_ARRAY, dh_a , nn_model, n_closest_obs):
        self.tensor_args = {'device': q0.device, 'dtype': q0.dtype}
        self.n_dof = q0.shape[0]
        self.Policy = TensorPolicyMPPI(N_traj, self.n_dof, self.tensor_args)
        self.q0 = q0
        self.DS_idx = 0
        self.DS_ARRAY = DS_ARRAY
        self.DS = DS_ARRAY[self.DS_idx]
        self.qf = self.DS.q_goal.squeeze()
        self.dh_params = dh_params
        self.obs = obs
        self.n_obs = obs.shape[0]
        self.dt = dt
        self.dt_H = dt_H
        self.N_traj = N_traj
        self.dh_a = dh_a
        self.nn_model = nn_model
        self.all_traj = torch.zeros(N_traj, dt_H, self.n_dof).to(**self.tensor_args)
        self.closest_dist_all = 100 + torch.zeros(N_traj, dt_H).to(**self.tensor_args)
        self.kernel_val_all = torch.zeros(N_traj, dt_H, self.Policy.N_KERNEL_MAX).to(**self.tensor_args)
        self.q_cur = q0
        self.nn_input = torch.zeros(N_traj * obs.shape[0], self.n_dof + 3).to(**self.tensor_args)
        self.D = (torch.zeros([self.N_traj, self.n_dof, self.n_dof]) + torch.eye(self.n_dof)).to(**self.tensor_args)
        self.D_tau = (torch.zeros([self.N_traj, self.n_dof, self.n_dof]) + torch.eye(self.n_dof)).to(**self.tensor_args)
        self.D_tau[:, 0, 0] = 0
        self.nn_grad = torch.zeros(N_traj, self.n_dof).to(**self.tensor_args)
        self.norm_basis = torch.zeros((self.N_traj, dt_H, self.n_dof, self.n_dof)).to(**self.tensor_args)
        self.dot_products = torch.zeros((self.N_traj, dt_H)).to(**self.tensor_args)

        self.basis_eye = torch.eye(self.n_dof).repeat(N_traj, 1).reshape(N_traj, self.n_dof, self.n_dof).to(**self.tensor_args).cpu()
        self.basis_eye_temp = (self.basis_eye * 0).to(**self.tensor_args).cpu()
        self.nn_model.allocate_gradients(self.N_traj+self.Policy.N_KERNEL_MAX, self.tensor_args)
        self.Cost = Cost(self.qf, self.dh_params)
        self.traj_range = torch.arange(self.N_traj).to(**self.tensor_args).to(torch.long)
        self.policy_upd_rate = 0.1
        self.dst_thr = 0.5
        self.qdot = torch.zeros((self.N_traj, self.n_dof)).to(**self.tensor_args)
        self.ker_thr = 1e-3
        self.ignored_links = [0, 1, 2]
        if self.n_dof <7:
            self.ignored_links = []
        self.kernel_gammas_tmp = torch.zeros(self.Policy.N_KERNEL_MAX, **self.tensor_args)
        self.kernel_obstacle_bases_tmp = torch.zeros((self.Policy.N_KERNEL_MAX, self.n_dof, self.n_dof), **self.tensor_args)
        self.n_closest_obs_to_check = n_closest_obs
        self.n_closest_obs = n_closest_obs

        for tmp in range(5):
            self.Policy.sample_policy()
            _, _, _, _ = self.propagate()
            numeric_fk_model(self.q_cur, dh_params, 10)
            print(f'warmup run #{tmp+1} done', flush=True)

    def reset_DS(self, DS):
        self.DS = DS
        self.qf = DS.q_goal.squeeze()
        self.Cost = Cost(self.qf, self.dh_params)

    def switch_DS_idx(self, idx):
        self.DS_idx = idx
        self.DS = self.DS_ARRAY[idx]
        self.qf = self.DS.q_goal.squeeze()
        self.Cost = Cost(self.qf, self.dh_params)

    def reset_tensors(self):
        self.all_traj = self.all_traj * 0
        self.closest_dist_all = 100 + self.closest_dist_all * 0
        self.kernel_val_all = self.kernel_val_all * 0
        self.dot_products = self.dot_products * 0
    def build_nn_input(self, q_tens, obs_tens):
        self.nn_input = torch.hstack((q_tens.tile(obs_tens.shape[0], 1), obs_tens.repeat_interleave(q_tens.shape[0], 0)))
        return self.nn_input

    def propagate(self):
        self.reset_tensors()
        self.all_traj[:, 0, :] = self.q_cur
        P = self.Policy
        for i in range(1, self.dt_H+1):
            with record_function("TAG: Nominal vector field"):
                q_prev = self.all_traj[:, i - 1, :]
                # calculate nominal vector field
                #nominal_velocity = (q_prev - self.qf) @ self.A
                nominal_velocity = self.DS.get_velocity(q_prev)
                nominal_velocity_norm = torch.norm(nominal_velocity, dim=1).reshape(-1, 1)
                nominal_velocity_normalized = nominal_velocity / nominal_velocity_norm

            #distance calculation (NN)
            with record_function("TAG: evaluate NN"):
                # evaluate NN. Calculate kernel bases on first iteration
                distance, self.nn_grad = self.distance_repulsion_nn(q_prev, aot=True)
                self.nn_grad = self.nn_grad[0:self.N_traj, :]               # fixes issue with aot_function cache
                # distance, self.nn_grad = self.distance_repulsion_fk(q_prev) #not implemented for Franka

                distance -= self.dst_thr
                self.closest_dist_all[:, i - 1] = distance[0:self.N_traj]
                distance = distance[0:self.N_traj]
            with record_function("TAG: QR decomposition"):
                # calculate modulations
                self.basis_eye_temp = self.basis_eye_temp*0 + self.basis_eye
                self.basis_eye_temp[:, :, 0] = self.nn_grad.cpu().to(torch.float32)
                E, R = torch.linalg.qr(self.basis_eye_temp)
                E = E.to(**self.tensor_args)
                E[:, :, 0] = self.nn_grad / self.nn_grad.norm(2, 1).unsqueeze(1)
                self.norm_basis[:, i-1] = E

                dotproduct = (E[:, :, 0] * nominal_velocity_normalized).sum(dim=-1)
                self.dot_products[:, i-1] = dotproduct
                l_vel = generalized_sigmoid(dotproduct, 0, 1, -0.2, 0.0, 100)
            with record_function("TAG: Modulation-propagation"):
                # calculate standard modulation coefficients
                # gamma = distance + 1
                # gamma[gamma < 0] = 1e-8
                # l_n = 1 - 1 / gamma
                # l_tau = 1 + 1 / gamma
                # l_n[l_n < 0] = 0
                # l_tau[l_tau < 1] = 1
                # calculate own modulation coefficients
                if 0:
                    # for planar robot (units)
                    dist_low, dist_high = 0.0, 2.5
                    k_sigmoid = 3
                else:
                    # for franka robot (meters)
                    dist_low, dist_high = 0.03, 0.1
                    k_sigmoid = 100
                ln_min, ln_max = 0, 1
                ltau_min, ltau_max = 1, 3
                l_n = generalized_sigmoid(distance, ln_min, ln_max, dist_low, dist_high, k_sigmoid)
                l_n_vel = l_vel * 1 + (1 - l_vel) * l_n
                l_tau = generalized_sigmoid(distance, ltau_max, ltau_min, dist_low, dist_high, k_sigmoid)
                # self.D = self.D * 0 + torch.eye(self.n_dof).to(**self.tensor_args)
                # self.D = l_tau[:, None, None] * self.D
                self.D = l_tau.repeat_interleave(self.n_dof).reshape((self.N_traj, self.n_dof)).diag_embed(0, 1, 2)
                self.D[:, 0, 0] = l_n_vel
                # build modulation matrix
                M = E @ self.D @ E.transpose(1, 2)

            # apply policies
            with record_function("TAG: Apply policies"):
                kernel_value = eval_rbf(q_prev, P.mu_tmp[:, 0:P.n_kernels], P.sigma_tmp[:, 0:P.n_kernels], P.p)
                self.ker_w = kernel_value
                #### important to nullify first component (to ensure tangential motion)
                # P.alpha_tmp[:, 0:P.n_kernels, 0] = 0
                #### that's for kernel gamma(q_k) policy
                #policy_all_flows = (P.kernel_obstacle_bases[0:P.n_kernels] @ P.alpha_tmp[:, 0:P.n_kernels].unsqueeze(3)).squeeze(-1)
                #### that's for local gamma(q) policy
                # policy_all_flows = torch.matmul(E[:, None], P.alpha_tmp[:, 0:P.n_kernels].unsqueeze(3)).squeeze(-1)
                #### disable tangential stuff, optimize just 7d vector field
                policy_all_flows = P.alpha_tmp[:, 0:P.n_kernels]

                # sum weighted inputs from kernels #### BUG HERE [FIXED FINALLY]!!!!
                policy_value = torch.sum(policy_all_flows * self.ker_w, 1)
                ## project it to tangent space (if needed)
                M_tau = E @ self.D_tau @ E.transpose(1, 2)
                #policy_value = (M_tau @ policy_value.unsqueeze(-1)).squeeze(-1)
                # for one kernel policy_all_flows [100, 7], ker_w [100, 1, 1]
                # valid multiplication
                if P.n_kernels > 0:
                    self.kernel_val_all[:, i - 1, 0:P.n_kernels] = kernel_value.reshape((self.N_traj, P.n_kernels))
                if P.n_kernels == 0:
                    policy_value = nominal_velocity * 0
            with record_function("TAG: Apply policy"):
                # policy control
                ### policy depends on kernel Gamma(q_k) - assuming matrix multiplication done above
                #policy_velocity = policy_value
                collision_activation = (1-l_n[:, None])
                velocity_activation = (1-l_vel[:, None])
                goal_activation = (q_prev-self.qf).norm(p=0.5, dim=1).clamp(0, 1).unsqueeze(1)
                goal_activation[goal_activation < 0.3] = 0
                policy_velocity = collision_activation * velocity_activation * goal_activation * policy_value
                # policy_velocity = velocity_activation * policy_value
                # if self.N_traj == 1:
                #     print(collision_activation.item(), velocity_activation.item())
                #policy_velocity += ((1-l_n)/100).unsqueeze(1) * E[:, :, 0] #(some repuslion tweaking)
                # calculate modulated vector field (and normalize)

                # TODO CHECK POLICY VELOCITY NORMALIZATION (it's way less than 1)
                policy_velocity = policy_velocity * nominal_velocity_norm             # magnitude of nominal velocity
                total_velocity = nominal_velocity + policy_velocity                   # magnitude of 2x nominal velocity
                # total_velocity_norm = torch.norm(total_velocity, dim=1).unsqueeze(1)
                # total_velocity_scaled = nominal_velocity_norm * total_velocity / total_velocity_norm
                mod_velocity = (M @ (total_velocity).unsqueeze(2)).squeeze()
                # # normalization
                mod_velocity_norm = torch.norm(mod_velocity, dim=-1).reshape(-1, 1)
                mod_velocity_norm[mod_velocity_norm <= 0.5] = 1
                mod_velocity = torch.nan_to_num(mod_velocity / mod_velocity_norm)
                # slow down and repulsion for collision case
                mod_velocity[distance < 0] *= 0.1
                repulsion_velocity = E[:, :, 0] * nominal_velocity_norm*0.2
                mod_velocity[distance < 0] += repulsion_velocity[distance < 0]
            with record_function("TAG: Propagate"):
                # propagate
                if i < self.dt_H:
                    self.all_traj[:, i, :] = self.all_traj[:, i - 1, :] + self.dt * mod_velocity
                if i == 1:
                    self.qdot = mod_velocity
        return self.all_traj, self.closest_dist_all, self.kernel_val_all[:, :, 0:P.n_kernels], self.dot_products


    def distance_repulsion_nn(self, q_prev, aot=False):
        n_inputs = q_prev.shape[0]
        with record_function("TAG: evaluate NN_1 (build input)"):
            # building input tensor for NN (N_traj * n_obs, n_dof + 3)
            nn_input = self.build_nn_input(q_prev, self.obs)

        with record_function("TAG: evaluate NN_2 (forward pass)"):
            # doing single forward pass to figure out the closest obstacle for each configuration
            nn_dist = self.nn_model.model_jit.forward(nn_input[:, 0:-1])
            if self.nn_model.out_channels == 9:
                nn_dist = nn_dist/100   # scale down to meters
        with record_function("TAG: evaluate NN_3 (get closest obstacle)"):
            # rebuilding input tensor to only include closest obstacles
            nn_dist -= nn_input[:, -1].unsqueeze(1)  # subtract radius
            nn_dist[:, self.ignored_links] = 1e6  # ignore specified links
            ## check there are enough obstacles
            if self.n_obs < self.n_closest_obs_to_check :
                self.n_closest_obs = self.n_obs

            mindist, _ = nn_dist.min(1)
            mindist_matrix = mindist.reshape(self.n_obs, n_inputs).transpose(0, 1)
            mindist, sphere_idx = mindist_matrix.min(1)
            sort_dist, sort_idx = mindist_matrix.sort(dim = 1)
            mindist_arr = sort_dist[:, 0:self.n_closest_obs]
            sphere_idx_arr = sort_idx[:, 0:self.n_closest_obs]
            #mask_idx = self.traj_range[:n_inputs] + sphere_idx * n_inputs
            mask_idx = torch.arange(n_inputs).to(**self.tensor_args) + sphere_idx * n_inputs
            #nn_input = nn_input[mask_idx, :]
            # print(f" Q PREV : {q_prev}, n_imputs : {n_inputs}")
            # print(f" sphere idx arr : {sphere_idx_arr}")
            # print(f" sphere idx arr shape : {sphere_idx_arr.shape}")
            # # print(f" sort idx : {sort_idx}")
            # # print(f" min_dist : {mindist}")
            # # print(f" matrix: {mindist_matrix}")
            # print(f"obs : {self.obs}")

            # temp = torch.arange(n_inputs).to(**self.tensor_args).unsqueeze(1).repeat(1, self.n_closest_obs)
            # print(f" temp : {temp}, shape {temp.shape}")
            
            new_mask_idx = torch.arange(n_inputs).to(**self.tensor_args).unsqueeze(1).repeat(1, self.n_closest_obs) + sphere_idx_arr * n_inputs
            nn_input = nn_input[new_mask_idx.flatten().to(torch.long), :]

        with record_function("TAG: evaluate NN_4 (forward+backward pass)"):
            # forward + backward pass to get gradients for closest obstacles
            # nn_dist, nn_grad, nn_minidx = self.nn_model.compute_signed_distance_wgrad(nn_input[:, 0:-1], 'closest')
            if aot:
                nn_dist, nn_grad, nn_minidx = self.nn_model.dist_grad_closest_aot(nn_input[:, 0:-1])
            else:
                nn_dist, nn_grad, nn_minidx = self.nn_model.dist_grad_closest(nn_input[:, 0:-1])
                nn_grad = nn_grad.squeeze(2)

            self.nn_grad = nn_grad[:nn_input.shape[0], 0:self.n_dof]
            if self.nn_model.out_channels == 9:
                nn_dist = nn_dist/100   # scale down to meters

        with record_function("TAG: evaluate NN_5 (process outputs)"):
            # cleaning up to get distances and gradients for closest obstacles
            nn_dist -= nn_input[:, -1].unsqueeze(1)  # subtract radius and some threshold
            #extract closest link distance
            nn_dist = nn_dist[torch.arange(self.n_closest_obs * n_inputs).unsqueeze(1), nn_minidx.unsqueeze(1)]
            # reshape to match n_traj x n_closest_obs
            nn_dist = nn_dist.reshape(n_inputs, self.n_closest_obs)
            self.nn_grad = self.nn_grad.reshape(n_inputs, self.n_closest_obs, self.n_dof)
            # weight gradients according to closest distance
            weighting = (-10 * nn_dist).softmax(dim=-1)
            self.nn_grad = torch.sum(self.nn_grad * weighting.unsqueeze(2), dim=1)
            # distance - mindist
            distance = nn_dist[:, 0]

        return distance, self.nn_grad

    def update_kernel_normal_bases(self):
        if self.Policy.n_kernels > 0:
            # reset tensors for current bases
            self.kernel_obstacle_bases_tmp *= 0
            self.kernel_obstacle_bases_tmp[0:self.Policy.n_kernels][:] += torch.eye(self.n_dof)

            # calculate gamma and repulsion for kernel centers
            dst, grad = self.distance_repulsion_nn(self.Policy.mu_c[0:self.Policy.n_kernels], aot=False)
            self.kernel_obstacle_bases_tmp[0:self.Policy.n_kernels][:, :, 0] = grad

            #perform qr decomposition
            E, R = torch.linalg.qr(self.kernel_obstacle_bases_tmp[0:self.Policy.n_kernels].cpu().to(torch.float32))
            E = E.to(**self.tensor_args)
            E[:, :, 0] = grad / grad.norm(2, 1).unsqueeze(1)
            # print((self.Policy.kernel_obstacle_bases[0:self.Policy.n_kernels] - E).norm(p=2)) #debug update difference

            # update policies kernel bases
            self.Policy.kernel_obstacle_bases[0:self.Policy.n_kernels] = E
        else:
            pass
        return 0

    def distance_repulsion_fk(self, q_prev):
        all_links, all_int_pts = numeric_fk_model_vec(q_prev, self.dh_params, 10)
        distance, idx_obs_closest, idx_links_closest, idx_pts_closest = get_mindist(all_links, self.obs)
        obs_pos_closest = self.obs[idx_obs_closest, 0:3].squeeze(1)
        int_points_closest = all_int_pts[self.traj_range.unsqueeze(1), idx_links_closest, idx_pts_closest].squeeze(1)
        rep_vec = lambda_rep_vec(q_prev, obs_pos_closest, idx_links_closest, int_points_closest, self.dh_a)
        self.nn_grad = rep_vec[:, 0:self.n_dof]
        return distance, rep_vec

    def get_cost(self):
        self.cur_cost = self.Cost.evaluate_costs(self.all_traj, self.closest_dist_all)
        return self.cur_cost

    def get_qdot(self, mode='best'):
        qdot = 0
        if mode == 'best':
            best_idx = torch.argmin(self.cur_cost)
            qdot = self.qdot[best_idx, :]
        elif mode == 'weighted':
            beta = self.cur_cost.mean() / 50
            w = torch.exp(-1 / beta * self.cur_cost)
            w = w / w.sum()
            qdot = torch.sum(w.unsqueeze(1) * self.qdot, dim=0)
        return qdot

    def shift_policy_means(self):
        beta = self.cur_cost.mean() / 50
        w = torch.exp(-1 / beta * self.cur_cost)
        w = w / w.sum()
        max_kernel_activation_each = self.kernel_val_all[:, :, 0:self.Policy.n_kernels].max(dim=1)[0]
        mean_kernel_activation_all = max_kernel_activation_each.mean(dim=0)
        update_mask = mean_kernel_activation_all > self.ker_thr
        print(f'Updating {sum(update_mask)} kernels!', flush=True)
        update_mask_base = self.kernel_val_all[0, :, 0:self.Policy.n_kernels].mean(dim=0) > self.ker_thr
        update_mask = update_mask_base * update_mask
        print(f'Updating (#2) {sum(update_mask)} kernels!', flush=True)
        self.Policy.update_policy(w, self.policy_upd_rate, update_mask)
        return 0, sum(update_mask)

    def update_obstacles(self, obs):
        self.obs = obs
        self.n_obs = obs.shape[0]
        return 0

def generalized_sigmoid(x, y_min, y_max, x0, x1, k):
    return y_min + (y_max - y_min) / (1 + torch.exp(k * (-x + (x0+x1)/2)))

